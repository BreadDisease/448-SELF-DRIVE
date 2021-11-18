# Run `launch` script to get everything going.

## Important
Start `remote` before starting `server.py`. Make sure to load web UI **after** `server.py` is running.

### Compile `remote.cpp`
`mpicxx -g -Wall -std=c++17 -O3 remote.cpp cc1100_raspi.cpp -o remote -lboost_system -lboost_mpi -lboost_serialization -lpthread -lzmq -lwiringPi`

### Run `remote`
`mpirun -np 2 remote`

### Run `server.py`
`python3 server.py`
