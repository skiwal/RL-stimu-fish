# RL fish

## simulation

### complie

``` bash

cd /home/ubuntu/code/RL-stimu-fish/simulation

rm -rf build

cmake -S . -B build \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build build -j$(nproc)

```

simulation/data/env/basic_river.scn