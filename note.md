run without vcpkg
```
nix develop
mkdir build
cd build
cmake .. --preset nix-minimal
cmake --build . --parallel 10
```