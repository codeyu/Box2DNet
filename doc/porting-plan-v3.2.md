# Box2DNet：Box2D v3.2 → .NET 10 移植计划

## 1. 背景与目标

| 项目 | 现状 |
| --- | --- |
| 本仓库 `Box2DNet` | 基于 Box2DX（Box2D v2.0 时代）的 C# 移植，2018 年后停止更新 |
| 上游 `C:\github\box2d` | Box2D **v3.2.0**（`a292912`），纯 C17，约 46k 行（含头文件） |
| 参考 `C:\github\Box2DSharp` | Box2D v3.0（2024-09 同步）的 C# 移植，net8.0 |
| 工具链 | .NET SDK 10.0.402（C# 14） |

v3 与 v2 是完全不同的引擎架构（基于 id 句柄的 C API、Solver Set、约束图着色、Soft Step 求解器、SIMD 宽约束、多线程任务系统），所以这次是**重新移植**，不是在旧代码上升级。

**目标**

1. 以 v3.2.0 为基准，**逐文件、逐函数**移植到 .NET 10（`net10.0`），行为与 C 版一致。
2. **验收标准是跨平台确定性哈希**：上游 `test/test_determinism.c` 的 FallingHinges 场景（单精度）期望 `sleepStep == 299`、`hash == 0x465F3DF4`。C# 版必须得到同样的值，单线程和多线程都要。
3. 移植上游单元测试（`test/*.c`），全部通过。
4. 只用托管代码，默认不需要 `unsafe`。性能用 struct 数组、`ref` 局部变量、`Span<T>`、`System.Runtime.Intrinsics` 来保证。

5. 重写图形化 Testbed，**UI 库 cimgui 升级到最新版**（见 P7）。

**本期不做（列入后续阶段）**：双精度大世界模式（`BOX2D_DOUBLE_PRECISION`）、Recording/Replay/Snapshot（上游标注为 experimental）、Tracy 性能埋点、NuGet 发布。

## 2. 关键设计决策

### 2.1 API 形态：保持 C API 一一对应

Box2D v3 的公开 API 是扁平的 C 函数加 id 句柄（`b2WorldId`、`b2BodyId`……）。为了让官方文档、示例和测试能直接对照，移植版保留这种形态：

| C | C# |
| --- | --- |
| `b2Vec2`、`b2BodyDef`、`b2WorldId` | `Vec2`、`BodyDef`、`WorldId`（去掉 `b2` 前缀，`namespace Box2DNet`） |
| `b2CreateWorld(&def)` | `B2.CreateWorld(in WorldDef def)` |
| `b2World_Step(id, dt, n)` | `B2.World_Step(id, dt, n)` |
| `b2Body_GetPosition(id)` | `B2.Body_GetPosition(id)` |
| `B2_LINEAR_SLOP`、`B2_PI` | `B2.LinearSlop`、`B2.Pi` |
| 结构体字段 `bodyDef.linearVelocity` | **保持原名** `bodyDef.linearVelocity` |

- 所有函数（公开和内部）都放在 `public static partial class B2` 里，按 C 源文件拆成多个 `partial` 文件。内部函数标为 `internal`。
- **字段名保持 C 的 camelCase**。这和 C# 的命名习惯不同，但它让 4 万行代码可以逐行对照，大幅降低移植错误，也方便以后跟进上游更新。
- 面向对象的薄包装（`World`、`Body` 等 struct 包住 id、提供实例方法）放到后续阶段，作为可选层。

### 2.2 内存模型

| C 用法 | C# 映射 |
| --- | --- |
| `b2Array(T)`（data/count/capacity 动态数组） | `struct B2Array<T>`，底层是 `T[]`，`ref T this[int]` 返回元素引用 |
| `b2Body* body = b2BodyArray_Get(...)` | `ref Body body = ref world.bodies[id];` |
| `b2World`、`b2BroadPhase`、`b2DynamicTree`、`b2ConstraintGraph` 等单例对象 | `sealed class`（只有一个实例，相当于指针语义） |
| `b2Body`、`b2Shape`、`b2Contact`、`b2BodySim` 等大量对象 | `struct`，放在 `B2Array<T>` 里，用 `ref` 访问 |
| 定长数组 `b2Vec2 vertices[8]`、`points[2]` | C# 12 `[InlineArray(N)]` 结构 |
| `b2Stack` 每步临时分配器 | `ArenaAllocator`：基于 `ArrayPool<T>` 的类型化租借，保留 LIFO 断言 |
| `void* userData` / `void* context` | `object?` |
| 函数指针回调 | 委托（`delegate`） |
| `b2DebugDraw` 函数指针结构 | `class DebugDraw`，字段是委托 |
| `b2AtomicInt` 等 | `int` 字段加 `Interlocked` / `Volatile` |
| `memset` / `memcpy` | `Array.Clear` / `Span.CopyTo` |

规则：和 C 一样，持有某个数组元素的 `ref` 期间不能让该数组扩容。C 版在这些地方本来就有同样的约束（指针会失效），所以照搬原代码的顺序即可。

### 2.3 数学与确定性

- `b2ComputeCosSin`、`b2Atan2`、`b2MakeRot` 等上游自研的确定性数学函数**逐位照搬**。`MathF.Sqrt` 是 IEEE 精确运算，可以直接用。
- 不使用 `MathF.FusedMultiplyAdd`，SIMD 部分也不用 FMA。RyuJIT 不会自动合并乘加，这和上游关闭 FP contraction 的做法一致。
- `b2Hash`（djb2）照搬。哈希的输入是 `WorldTransform` 的原始字节，用 `MemoryMarshal.AsBytes` 取得，和 C 的内存布局一致（`[StructLayout(Sequential)]`）。

### 2.4 SIMD

`contact_solver_wide.inl` 是图着色后的宽约束接触求解器，有 4 宽和 8 宽两种。

- 4 宽：用 `Vector128<float>`（x64 上走 SSE2，ARM64 上走 NEON，没有硬件支持时 .NET 有软件回退）。
- 8 宽：`Vector256.IsHardwareAccelerated` 为真时用 `Vector256<float>`（对应上游运行时 AVX2 分派）。
- 只使用 add/sub/mul/div/min/max/compare/select 这些逐元素运算，保证和标量路径结果逐位一致。确定性测试会同时跑两种宽度来验证。

### 2.5 多线程

- `scheduler.c`（v3.2 新增的内置线程池）：用 .NET `Thread` 和 `SemaphoreSlim` 移植，语义不变（没有挂接外部任务回调时，Box2D 自己起线程）。
- `EnqueueTaskCallback` / `FinishTaskCallback` 保留为委托，用户可以接入自己的任务系统（例如 `ThreadPool`）。
- `parallel_for.c` 和 solver 里基于 CAS 的块认领机制，用 `Interlocked.CompareExchange` 照搬。
- `workerCount = 1` 时完全单线程运行，这也是调试的默认配置。

### 2.6 断言与输入校验

- `B2_ASSERT` → `[Conditional("DEBUG")]` 的 `B2.Assert(bool)`，可以通过 `B2.SetAssertCallback` 替换。
- `B2_CHECK_INPUT` / `B2_CHECK_INPUT_RETURN`：Release 下也生效，先记日志再返回（和 v3.2 行为一致）。
- `B2_VALIDATE` → `[Conditional("BOX2D_VALIDATE")]`。

## 3. 解决方案结构

```
Box2DNet.slnx
src/Box2DNet/                Box2DNet.csproj（net10.0，核心库）
  Base/        core, constants, id, math, containers, arena, id_pool, bitset, table, timer
  Collision/   aabb, hull, geometry, distance, manifold, dynamic_tree, mover
  Dynamics/    world, body, shape, chain, contact, sensor, island, solver_set,
               constraint_graph, broad_phase, solver, contact_solver(+wide)
  Joints/      joint, distance, motor, mover, pogo, prismatic, revolute, weld, wheel
  Threading/   scheduler, parallel_for
tests/Box2DNet.Tests/        移植自上游 test/*.c
examples/Testbed/            Hexa.NET.GLFW + OpenGL 3.3 + Hexa.NET.ImGui（最新 cimgui）图形化示例
legacy/                      旧的 v2 代码（git mv 过来保存，不参与构建）
```

旧 v2 代码先用分支 `v2-legacy` 保留一份，再移到 `legacy/` 目录，不删除任何历史。

## 4. 文件映射与工作量

（行数是 C 源码行数，含注释）

| 阶段 | C 源文件 | C# 文件 | 行数 |
| --- | --- | --- | --- |
| P1 基础 | `base.h` `config.h` `constants.h` `id.h` `math_types.h` `math_functions.h/.c` `core.h/.c` `container.h` `arena_allocator` `id_pool` `bitset` `table` `timer`（部分） | `Base/*.cs` | ~2.5k |
| P2 碰撞 | `collision.h` `aabb` `hull` `geometry` `distance` `manifold` `dynamic_tree` `mover` | `Collision/*.cs` | ~9k |
| P3 世界 | `types.h/.c` `box2d.h` `physics_world` `body` `shape` `contact` `sensor` `island` `solver_set` `constraint_graph` `broad_phase` | `Dynamics/*.cs` | ~14k |
| P4 求解器 | `solver` `contact_solver` `contact_solver_wide.inl` `simd` `parallel_for` `scheduler` | `Dynamics/Solver*.cs` `Threading/*.cs` | ~5.5k |
| P5 关节 | `joint` + 8 种关节 | `Joints/*.cs` | ~5.2k |
| P6 测试/示例 | `test/*.c`（不含 recording/snapshot）、`shared/determinism.c` `benchmarks.c` | `tests/` `examples/` | ~7k |
| 后续 | `recording` `replay` `snapshot` 双精度 Testbed | — | ~6.4k |

## 5. 分阶段步骤

**P0 准备**：建 `v2-legacy` 分支；旧代码移到 `legacy/`；新建 `Box2DNet.slnx`、核心库和测试工程；配置 `Nullable`、`TreatWarningsAsErrors`（只对新代码）、`BOX2D_VALIDATE` 调试常量。

**P1 基础层**：数学类型和函数（含确定性三角函数）、id 结构和打包/解包、`B2Array<T>`、`IdPool`、`BitSet`、`HashSet`（`table.c` 的开放寻址集合）、`ArenaAllocator`、断言/日志/计时。完成后移植 `test_math`、`test_bitset`、`test_table`、`test_id`、`test_container`。

**P2 碰撞层**：凸包（quickhull）、几何体（圆、胶囊、多边形、线段、链段）的质量/包围盒/射线/形状投射；GJK 距离、形状投射、TOI；全部 manifold 函数；动态 AABB 树（含 v3.2 新的 refit/rebuild 和 `CastRay`、`CastBox`）；角色移动器的平面求解。完成后移植 `test_collision`、`test_distance`、`test_shape`、`test_dynamic_tree`、`test_mover`。

**P3 世界层**：默认定义（`types.c`）、世界创建/销毁/查询/事件、刚体、形状、链形状、接触（含 v3.2 的接触复用）、传感器（含连续传感器事件）、岛的合并/拆分、Solver Set 休眠/唤醒、约束图着色、宽相。

**P4 求解器**：Soft Step 主循环、各求解阶段、标量 overflow 求解器、4/8 宽 SIMD 求解器、连续碰撞（TOI/bullet）、恢复系数迭代、内置调度器和 parallel-for。

**P5 关节**：通用关节逻辑（局部坐标系、断裂事件、约束软度）以及 distance、motor（含弹簧和速度目标）、mover、pogo、prismatic、revolute、weld、wheel。

**P6 验证**：
1. 移植 `test_world`、`test_chain`、`test_restitution`、`test_invalid_input`、`test_large_world`（单精度部分）、`test_thread`、`test_determinism`。
2. **确定性验收**：FallingHinges 场景在 `workerCount = 1/2/4/8`、SIMD 宽度 4/8 下都得到 `sleepStep = 299`、`hash = 0x465F3DF4`。
3. HelloWorld（上游 `docs/hello.md`）的结果一致（以单元测试形式实现）。
4. 补一个简单 benchmark（上游 `shared/benchmarks.c` 里的 large pyramid 等）用于性能回归。

**P7 Testbed 与 cimgui 升级**：
- 旧版 Testbed 用 Veldrid，并把 2018 年的 cimgui 原生库（`deps/cimgui/*`）直接提交进仓库。新版不再提交二进制文件，改为通过 NuGet 引用最新的 cimgui/Dear ImGui：**Hexa.NET.ImGui**（跟随 cimgui 最新版本，自带 win/linux/osx 的 x64 和 arm64 原生库）。
- 窗口和渲染用 **Hexa.NET.GLFW** + OpenGL 3.3（ImGui 的 GLFW/OpenGL3 后端来自 Hexa.NET.ImGui.Backends），对应上游 samples 的 GLFW + OpenGL 方案。调试绘制直接画在 ImGui 的背景绘制列表上，不需要自己写着色器。
- 移植上游 `samples/draw.c` 的调试绘制（实现 `DebugDraw` 委托）、`sample.cpp` 的框架（暂停、单步、鼠标拖拽、设置面板），再移植一部分代表性示例（HelloWorld、堆叠、关节、链条、汽车、确定性）。

**后续阶段（本期之外）**：Recording/Replay/Snapshot；双精度模式（`Pos`/`WorldTransform` 换成 double 版本，作为单独的编译常量和程序集）；面向对象包装层；移植上游全部 samples；NuGet 打包和 CI。

## 6. 风险与对策

| 风险 | 对策 |
| --- | --- |
| 规模大（约 4 万行），手工移植容易出错 | 字段名和函数结构与 C 保持一致；每完成一层就移植对应测试；最后用确定性哈希兜底 |
| 浮点结果和 C 版有细微差别，导致哈希不一致 | 不用 FMA；数学函数逐位照搬；对照 C 版运算顺序；不行时逐步打印中间状态，和 C 版对比来二分定位（需要本地 C 编译器，目前机器上没有，必要时安装 CMake + MSVC/clang） |
| 托管数组带来的边界检查开销 | 热路径用 `ref` + `Unsafe.Add`/`MemoryMarshal.GetArrayDataReference`（不需要 `unsafe` 关键字）；先保证正确，再做性能优化 |
| `ref` 指向的元素因数组扩容失效 | 照搬 C 的执行顺序（C 版有同样的约束）；Debug 版 `B2Array` 记录版本号来检测 |
| 多线程下的数据竞争 | 先用单线程跑通全部测试，再开多线程；确定性测试覆盖多种 worker 数 |
| v3.2 中带 experimental 标记的 API 可能变化 | 本期不移植 Recording/Snapshot；mover/pogo 关节照常移植 |

## 7. 完成定义

- `dotnet build` 零警告（新代码），`dotnet test` 全部通过。
- 确定性测试在所有线程数和 SIMD 宽度配置下哈希一致，并且等于上游期望值。
- `README.md` 更新：说明新版基于 Box2D v3.2.0、API 对应规则、快速上手示例，并指向旧版分支。

## 8. 当前进度（2026-10-26）

| 阶段 | 状态 |
| --- | --- |
| P0 准备 | 完成。旧代码在 `v2-legacy` 分支和 `legacy/` 目录 |
| P1 基础层 | 完成 |
| P2 碰撞层 | 完成 |
| P3 世界层 | 完成 |
| P4 求解器 | 完成。只移植了 4 宽 SIMD 求解器（`Vector128`，SSE2 语义），8 宽 AVX2 版本未移植；两者结果完全一致 |
| P5 关节 | 完成 |
| P6 验证 | 大部分完成。见下 |
| P7 Testbed | 完成。移植了上游全部 145 个 sample（不含 Replay，以及依赖 snapshot 的 Rollback），框架对应 `sample.h`/`draw.h`/`main.cpp` |

**测试**：`dotnet test` 全部通过，包括上游的 collision、distance、math、shape、bitset、mover、world（含依赖 junkyard benchmark 的 TestSetWorkerCount）测试，以及 HelloWorld。测试类之间不能并行（世界注册表是全局的，和上游一样单线程跑测试），由 `xunit.runner.json` 关闭。
**shared**：上游 `shared/` 移植为 `shared/Box2DNet.Shared`（benchmarks、human、determinism、utils），测试和 Testbed 共用。
**Samples**：用 `tools/c2cs/cpp2cs.py` 把 C++ sample 机械转换后手工修正（`tools/c2cs/patches/samples_*.py` 记录了手工修正）。`Testbed --smoke` 逐个运行全部 sample，目前全部通过。移植 sample 时发现并修复了库里 `World_OverlapShape` 的 bug（ref 字段没有用 ref 赋值），并加了回归测试。
**确定性验收通过**：FallingHinges 在 worker 数 0～32、内置调度器、跨平台变体下都得到 `sleepStep = 299`、`hash = 0x465F3DF4`，和 C 版逐位一致。

**尚未完成**：
- 还没移植的测试（在 `tools/c2cs/pending/`）：dynamic_tree、invalid_input、thread、sort、id、restitution、chain、large_world。
- Samples：Replay（依赖 recording）和 Rollback（依赖 snapshot）。
- Recording/Replay/Snapshot、双精度模式、8 宽 AVX2 求解器、性能 benchmark。
- `World_GetStateHash` 依赖 snapshot.c，测试里暂时用刚体变换的哈希代替。
