run without vcpkg

```
nix develop
mkdir build
cd build
cmake .. --preset nix-minimal
cmake --build . --parallel 10
```

```
./scripts/roofer_complexity_experiment.sh --spawn --filter "identificatie = '503100000000296'" ./data/wippolder/wippolder.las ./data/wippolder/wippolder.gpkg ./out
```

```
./build/apps/roofer-app/roofer --rerun --filter "identificatie = '503100000000296'" --complexity-factor 0.5  ./data/wippolder/wippolder.las ./data/wippolder/wippolder.gpkg ./out
```
