run without vcpkg

```
nix develop
mkdir build
cd build
cmake .. --preset nix-minimal
cmake --build . --parallel 10
```

```
./scripts/roofer_complexity_experiment.sh --filter "identificatie = '503100000000296'" ./data/wippolder/wippolder.503100000030812las ./data/wippolder/wippolder.gpkg ./out
```

```
./build/apps/roofer-app/roofer --rerun --filter "identificatie = '503100000000296'" --complexity-factor 0.5  ./data/wippolder/wippolder.las ./data/wippolder/wippolder.gpkg ./out
```

```
./run_roofer_configs.sh --filter "identificatie IN ('503100000000296', '503100000030812', '503100000032817')" ../roofer/data/wippolder/wippolder.las ../roofer/data/wippolder/wippolder.gpkg ./out ./config true
```

### Benchmark buildings

- 503100000000296 : building with dormers
- 503100000030812 : building looks like church
- 503100000032817 : building connected to another building
