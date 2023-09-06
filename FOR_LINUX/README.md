# RC Query API

This API demonstrates connection between DH GPU CModel and the Bolt ray traversal engine CModel. In this API, the GPU shader acts as the master to initiate any ray query request from any shader stage and retrieve the output from the traversal result to fulfill all Vulkan ray query intersection data requests.

## Build Instructions

```
g++ rc_query_api.cpp -o rc_query
```

## Runtime Instructions

```
./rc_query
```

## Licensing

This repository is PRIVATE for usage only by authorized employees of Bolt Graphics, Inc.

(c) 2023 Bolt Graphics, Inc.
