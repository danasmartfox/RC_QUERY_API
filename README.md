# RC Query API

This API demonstrates connection between DH GPU CModel and the Bolt ray traversal engine CModel. In this API, the GPU shader acts as the master to initiate any ray query request from any shader stage and retrieve the output from the traversal result to fulfill all Vulkan ray query intersection data requests.

## Build Instructions

This project is compatible with the default Visual Studio 2019 (MSVC) development environment in machines running Windows OS. To create a local copy, clone the repository in VS 2019.

An alternative way is to download the .cpp and .hpp files from this repository, create an empty C++ project in VS 2019, and copy the .cpp, .hpp, and (sample logs) .txt files to the created project's Source Files.

## Runtime Instructions

To test the API in VS 2019, select Debug and Start Debugging.

## Licensing

This repository is PRIVATE for usage only by authorized employees of Bolt Graphics, Inc.

(c) 2023 Bolt Graphics, Inc.
