#include <iostream>
#include <cstdlib>
#include "rc_query.hpp"
using namespace std;

void RC_Query(
    RayData* VSI_indata,
    QueryResultData* VSI_outdata,
    unsigned int threadID,
    void* AS_VIR_ADDR,
    unsigned int cmdQuery,
    unsigned int* statusQuery,
    bool debugMessage
    //READ_MEMORY_CB mmu_read,
    //WRITE_MEMORY_CB mmu_write
) {
    unsigned int RayObjOrigin_com[3];
    unsigned int RayObjDirection_com[3];
    unsigned int TriBarycentric_com[2];
    unsigned int beta_com = 0;
    unsigned int gamma_com = 0;
    unsigned int THit_com = 0;
    unsigned int InstanceId_com = 0;
    unsigned int GeoId_com = 0;
    unsigned int PrimId_com = 0;
    unsigned int WorldToObjMatrix_com[12];
    unsigned int Type_com = 0;
    unsigned int Frontface_com = 0;
    unsigned int AABBopaque_com = 0;
    unsigned int CustomIndex_com = 0;
    unsigned int InstanceSbtOffset_com = 0;

    unsigned int RayObjOrigin_can[3];
    unsigned int RayObjDirection_can[3];
    unsigned int TriBarycentric_can[2];
    unsigned int beta_can = 0;
    unsigned int gamma_can = 0;
    unsigned int THit_can = 0;
    unsigned int InstanceId_can = 0;
    unsigned int GeoId_can = 0;
    unsigned int PrimId_can = 0;
    unsigned int WorldToObjMatrix_can[12];
    unsigned int Type_can = 0;
    unsigned int Frontface_can = 0;
    unsigned int AABBopaque_can = 0;
    unsigned int CustomIndex_can = 0;
    unsigned int InstanceSbtOffset_can = 0;

    // INSERT PROCESSES FOR: ///////////////////////////////////////////////////////////////////////////////////////////////
    //  unsigned int threadID, /////////////////////////////////////////////////////////////////////////////////////////////
    //  void* AS_VIR_ADDR, /////////////////////////////////////////////////////////////////////////////////////////////////
    //  unsigned int cmdQuery, /////////////////////////////////////////////////////////////////////////////////////////////
    //  unsigned int* statusQuery, /////////////////////////////////////////////////////////////////////////////////////////
    //  bool debugMessage, /////////////////////////////////////////////////////////////////////////////////////////////////
    //  READ_MEMORY_CB mmu_read, ///////////////////////////////////////////////////////////////////////////////////////////
    //  WRITE_MEMORY_CB mmu_write //////////////////////////////////////////////////////////////////////////////////////////





    // INSERT COMPUTATIONS HERE THAT UTILIZE VSI_in TO DERIVE VALUES FOR VSI_out_com (COMMITTED) AND VSI_out_can (CANDIDATE)
    // PASS THE COMPUTATION RESULTS TO THE VARIABLES ABOVE /////////////////////////////////////////////////////////////////
    // Sample only (to verify correct passing of results) //////////////////////////////////////////////////////////////////

    RayObjOrigin_com[0] = rand();
    RayObjOrigin_com[1] = rand();
    RayObjOrigin_com[2] = rand();
    RayObjOrigin_can[0] = rand();
    RayObjOrigin_can[1] = rand();
    RayObjOrigin_can[2] = rand();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    IntersectionData VSI_out_com;
    IntersectionData VSI_out_can;

    VSI_out_com.rayObjOrigin[0] = RayObjOrigin_com[0];
    VSI_out_com.rayObjOrigin[1] = RayObjOrigin_com[1];
    VSI_out_com.rayObjOrigin[2] = RayObjOrigin_com[2];
    VSI_out_com.rayObjDirection[0] = RayObjDirection_com[0];
    VSI_out_com.rayObjDirection[1] = RayObjDirection_com[1];
    VSI_out_com.rayObjDirection[2] = RayObjDirection_com[2];
    VSI_out_com.triBarycentric[0] = TriBarycentric_com[0];
    VSI_out_com.triBarycentric[1] = TriBarycentric_com[1];
    VSI_out_com.Beta = beta_com;
    VSI_out_com.Gamma = gamma_com;
    VSI_out_com.tHit = THit_com;
    VSI_out_com.instanceId = InstanceId_com;
    VSI_out_com.geoId = GeoId_com;
    VSI_out_com.primId = PrimId_com;
    VSI_out_com.worldToObjMatrix[0] = WorldToObjMatrix_com[0];
    VSI_out_com.worldToObjMatrix[1] = WorldToObjMatrix_com[1];
    VSI_out_com.worldToObjMatrix[2] = WorldToObjMatrix_com[2];
    VSI_out_com.worldToObjMatrix[3] = WorldToObjMatrix_com[3];
    VSI_out_com.worldToObjMatrix[4] = WorldToObjMatrix_com[4];
    VSI_out_com.worldToObjMatrix[5] = WorldToObjMatrix_com[5];
    VSI_out_com.worldToObjMatrix[6] = WorldToObjMatrix_com[6];
    VSI_out_com.worldToObjMatrix[7] = WorldToObjMatrix_com[7];
    VSI_out_com.worldToObjMatrix[8] = WorldToObjMatrix_com[8];
    VSI_out_com.worldToObjMatrix[9] = WorldToObjMatrix_com[9];
    VSI_out_com.worldToObjMatrix[10] = WorldToObjMatrix_com[10];
    VSI_out_com.worldToObjMatrix[11] = WorldToObjMatrix_com[11];
    VSI_out_com.type = Type_com;
    VSI_out_com.frontface = Frontface_com;
    VSI_out_com.AABBOpaque = AABBopaque_com;
    VSI_out_com.customIndex = CustomIndex_com;
    VSI_out_com.instanceSbtOffset = InstanceSbtOffset_com;

    VSI_out_can.rayObjOrigin[0] = RayObjOrigin_can[0];
    VSI_out_can.rayObjOrigin[1] = RayObjOrigin_can[1];
    VSI_out_can.rayObjOrigin[2] = RayObjOrigin_can[2];
    VSI_out_can.rayObjDirection[0] = RayObjDirection_can[0];
    VSI_out_can.rayObjDirection[1] = RayObjDirection_can[1];
    VSI_out_can.rayObjDirection[2] = RayObjDirection_can[2];
    VSI_out_can.triBarycentric[0] = TriBarycentric_can[0];
    VSI_out_can.triBarycentric[1] = TriBarycentric_can[1];
    VSI_out_can.Beta = beta_can;
    VSI_out_can.Gamma = gamma_can;
    VSI_out_can.tHit = THit_can;
    VSI_out_can.instanceId = InstanceId_can;
    VSI_out_can.geoId = GeoId_can;
    VSI_out_can.primId = PrimId_can;
    VSI_out_can.worldToObjMatrix[0] = WorldToObjMatrix_can[0];
    VSI_out_can.worldToObjMatrix[1] = WorldToObjMatrix_can[1];
    VSI_out_can.worldToObjMatrix[2] = WorldToObjMatrix_can[2];
    VSI_out_can.worldToObjMatrix[3] = WorldToObjMatrix_can[3];
    VSI_out_can.worldToObjMatrix[4] = WorldToObjMatrix_can[4];
    VSI_out_can.worldToObjMatrix[5] = WorldToObjMatrix_can[5];
    VSI_out_can.worldToObjMatrix[6] = WorldToObjMatrix_can[6];
    VSI_out_can.worldToObjMatrix[7] = WorldToObjMatrix_can[7];
    VSI_out_can.worldToObjMatrix[8] = WorldToObjMatrix_can[8];
    VSI_out_can.worldToObjMatrix[9] = WorldToObjMatrix_can[9];
    VSI_out_can.worldToObjMatrix[10] = WorldToObjMatrix_can[10];
    VSI_out_can.worldToObjMatrix[11] = WorldToObjMatrix_can[11];
    VSI_out_can.type = Type_can;
    VSI_out_can.frontface = Frontface_can;
    VSI_out_can.AABBOpaque = AABBopaque_can;
    VSI_out_can.customIndex = CustomIndex_can;
    VSI_out_can.instanceSbtOffset = InstanceSbtOffset_can;

    VSI_outdata->committed = VSI_out_com;
    VSI_outdata->candidate = VSI_out_can;
}

int main() {
    // TEST ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // THIS MODELS THE CREATION OF INPUTS TO RC_Query FUNCTION, WITH THE GPU SHADER FILLING UP THE DATA FOR VSI_indata /////

    RayData VSI_indata[100]; ////////////////////
    QueryResultData VSI_outdata[100]; ///////////
    //unsigned int threadID[100];
    void* AS_VIR_ADDR [100]; 
    unsigned int cmdQuery[100]; 
    unsigned int statusQuery[100]; //////////////
    bool debugMessage[100]; 
    
    for (int i = 0; i < 100; i++) {
        RC_Query(&VSI_indata[i], &VSI_outdata[i], i, AS_VIR_ADDR[i], i, &statusQuery[i], true);
        cout << i << ": [CANDIDATE] "
            << VSI_outdata[i].candidate.rayObjOrigin[0] << ", "
            << VSI_outdata[i].candidate.rayObjOrigin[1] << ", "
            << VSI_outdata[i].candidate.rayObjOrigin[2] << "      [COMMITTED] "
            << VSI_outdata[i].committed.rayObjOrigin[0] << ", "
            << VSI_outdata[i].committed.rayObjOrigin[1] << ", "
            << VSI_outdata[i].committed.rayObjOrigin[2] << endl;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    cout << "SUCCESS!" << endl; // DEBUG MESSAGE ONLY
    return 0;
}