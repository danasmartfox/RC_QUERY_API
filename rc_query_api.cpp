#include <iostream>
#include <cstdlib>
#include <vector>
#include "rc_query.hpp"
using namespace std;

bool extents_intersect (
    float num[7],
    float den[7],
    float ed0[7],
    float ed1[7],
    float& t_near,
    float& t_far
) {
    /* Code to do extents intersect here */
    for (int i = 0; i < 7; i++) {
        if (den[i] == 0) {
            continue;
        }

        /* Swap values using mux, should be similar to above code */
        float t_near_extents = (den[i] < 0)
            ? ((ed1[i] - num[i]) / den[i])
            : ((ed0[i] - num[i]) / den[i]);
        float t_far_extents = (den[i] < 0)
            ? ((ed0[i] - num[i]) / den[i])
            : ((ed1[i] - num[i]) / den[i]);

        t_near = (t_near_extents > t_near) ? t_near_extents : t_near;
        t_far = (t_far_extents < t_far) ? t_far_extents : t_far;

        if (t_near > t_far) {
            break;
        }
    }

    return (t_near > t_far) ? false : true;
}

int dot (vector<float> v1, vector<float> v2) {
    float product = 0;
    for (int i = 0; i < v1.size(); i++)
        product += v1[i] * v2[i];
    return product;
}

void precomp (
    vector<float> in,
    float out[7]
) {
    float pre[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    float K0 = 0.0;
    float K1 = 1.0;
    float K2 = sqrt(3) / 3.0;

    vector<float> plane_normal[7] = {
        vector<float> { K1, K0, K0 },
        vector<float> { K0, K1, K0 },
        vector<float> { K0, K0, K1 },
        vector<float> { K2, K2, K2 },
        vector<float> { -K2, K2, K2 },
        vector<float> { -K2, -K2, K2 },
        vector<float> { K2, -K2, K2 }
    };

    for (int i = 0; i < 7; i++) {
        out[i] = dot(plane_normal[i], in); //(i+1.0)/10.0;
    }
}

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

    // PRE-PROCESSING //////////////////////////////////////////////////////////////////////////////////////////////////////
    // PRECOMPUTE ORIG AND DIR TO CREATE PRENUMERATOR AND PREDENOMINATOR ///////////////////////////////////////////////////
    vector<float> orig_v{ float(VSI_indata->rayOrigin[0]), float(VSI_indata->rayOrigin[1]), float(VSI_indata->rayOrigin[2]) };
    vector<float> dir_v{ float(VSI_indata->rayDirection[0]), float(VSI_indata->rayDirection[1]), float(VSI_indata->rayDirection[2]) };
    
    float prenum[7];
    float preden[7];

    precomp(orig_v, prenum);
    precomp(dir_v, preden);

    cout << "prenum: " << prenum[0] << ", " << prenum[1] << ", " << prenum[2] << ", " << prenum[3] << ", " << prenum[4] << ", " << prenum[5] << ", " << prenum[6] << endl;

    // PROCESSING //////////////////////////////////////////////////////////////////////////////////////////////////////////
    uint8_t t_entry;
    uint32_t t_count;
    float t_ed0[7], t_ed1[7];                   // extents data
    vector<float> t_vn0, t_vn1, t_vn2;          // vertex normals
    vector<float> t_vx0, t_vx1, t_vx2;          // vertex

    float smallest_t;                           // need to write smallest_t instead of ports->t to buffer
    float output_u;
    float output_v;
    vector<float> output_n;
    uint32_t output_mid;
    uint32_t output_hit;                        // reset so it doesnt write every time

    float t_near = 0.0;                         // set t_near = 0 
    float t_far = 32767.9999847412109375;       // max value of Q16.16
    smallest_t = 32767.9999847412109375;        // infinity, same as t_far initial value

    t_near = 0.0;                               // reset t_near and t_far values
    t_far = 32767.9999847412109375;             // reset t_near and t_far values

    int e_hit = 0;                              // reset e_hit
    int t_hit = 0;                              // reset output_hit
    smallest_t = t_far;

    // TEST SCENE //////////////////////////////////////////////////////////////////////////////////////////////////////////
    e_hit = extents_intersect(prenum, preden, t_ed0, t_ed1, t_near, t_far);
    // if !e_hit proceed to test next ray, start from the start of triangle buffer
    // Add code on how to prompt RC_Query to accept next ray
    if (e_hit) {
        cout << "Test next ray, and start from the start of triangle buffer." << endl;
    }

    // TEST CHILD //////////////////////////////////////////////////////////////////////////////////////////////////////////
    else {
        t_near = 0.0;                           // reset t_near and t_far values
        t_far = 32767.9999847412109375;         // reset t_near and t_far values
        e_hit = extents_intersect(prenum, preden, t_ed0, t_ed1, t_near, t_far);
    }

    // TEST TRIANGLES //////////////////////////////////////////////////////////////////////////////////////////////////////
    

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

    RayData VSI_indata; ////////////////////
    QueryResultData VSI_outdata; ///////////
    //unsigned int threadID[100];
    void* AS_VIR_ADDR = 0; 
    unsigned int cmdQuery = 0; 
    // Temporary cmdQuery values ///////////
    // 0 : AQRTQUERY_start_traversal
    // 1 : AQRTQUERY_generate_intersection
    // 2 : AQRTQUERY_continue_next_traversal
    // 3 : AQRTQUERY_terminate_traversal
    unsigned int statusQuery; //////////////
    bool debugMessage = false; 

    // FOR COMPUTATIONS ONLY ///////////////////////////////////////////////////////////////////////////////////////////////
    VSI_indata.rayOrigin[0] = 50;
    VSI_indata.rayOrigin[1] = 50;
    VSI_indata.rayOrigin[2] = 50;

    VSI_indata.rayDirection[0] = 100;
    VSI_indata.rayDirection[1] = 100;
    VSI_indata.rayDirection[2] = 100;

    RC_Query(&VSI_indata, &VSI_outdata, 0, &AS_VIR_ADDR, 0, &statusQuery, true);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    cout << "SUCCESS!" << endl; // DEBUG MESSAGE ONLY
    return 0;
}