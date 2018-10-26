1. Download [vulkansdk-macos-1.0.xx.0.tar.gz](https://vulkan.lunarg.com/sdk/home#sdk/downloadConfirm/latest/mac/vulkan-sdk.tar.gz)
2. `tar -xzf vulkan-sdk.tar.gz`
3. set environment variables(in `~/.profile` file):
```bash
export VK_SDK=~/vulkansdk-macos-1.1.85.0/macOS
export PATH=$VK_SDK/bin:$PATH
export DYLD_LIBRARY_PATH=$VK_SDK/lib:$DYLD_LIBRARY_PATH
export VK_LAYER_PATH=$VK_SDK/etc/vulkan/explicit_layers.d
export VK_ICD_FILENAMES=$VK_SDK/etc/vulkan/icd.d/MoltenVK_icd.json
```