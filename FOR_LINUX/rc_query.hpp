#include <iostream>
#include <cstring>
using namespace std;

typedef struct IntersectionData {
    unsigned int rayObjOrigin[3];
    unsigned int rayObjDirection[3];

    union {
        unsigned int triBarycentric[2];
        struct {
            unsigned int Beta : 32;
            unsigned int Gamma : 32;
        };
    };

    unsigned int tHit;
    unsigned int instanceId;
    unsigned int geoId;
    unsigned int primId;
    unsigned int worldToObjMatrix[12];
    unsigned int type : 2;
    unsigned int frontface : 1;
    unsigned int AABBOpaque : 1;
    unsigned int customIndex;
    unsigned int instanceSbtOffset;
}IntersectionData;

typedef struct QueryResultData {
    IntersectionData committed;
    IntersectionData candidate;
}QueryResultData;

typedef struct RayData {
    union {
        unsigned long long tlas_addr;
        struct {
            unsigned int tlas_addr_lower : 32;
            unsigned int tlas_addr_upper : 32;
        };
    };

    unsigned int tMin;
    unsigned int tMax;
    unsigned int rayDirection[3];
    unsigned int rayOrigin[3];
    unsigned int rayFlags;
    unsigned int cullMask;
    unsigned int tHit;
} RayData;

typedef struct RC_Query {
    struct RayData VSI_indata;
    struct QueryResultData VSI_outdata;
    unsigned int threadID;
    void* AS_VIR_ADDR;
    unsigned int cmdQuery;
    unsigned int* statusQuery;
    bool debugMessage;
    //READ_MEMORY_CB mmu_read;
    //WRITE_MEMORY_CB mmu_write;
} RC_Query;