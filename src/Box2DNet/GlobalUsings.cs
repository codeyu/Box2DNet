// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// In single precision the world position and world transform types are aliases, matching
// box2d/math_types.h. These keep the ported code close to the C source.
global using Pos = Box2DNet.Vec2;
global using WorldTransform = Box2DNet.Transform;
global using WorldCastOutput = Box2DNet.CastOutput;
