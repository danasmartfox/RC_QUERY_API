//
// Taken from original asbuild.cpp reference
//

#include <cstdint>
#include "VK_AS.hpp"

#if VK_USE_GPU_MMU
#define GetDeviceAddress(memory, offset)  (memory->devAddr + offset + 63) & 0xFFFFFFFFFFFFFFC0
#else
#define GetDeviceAddress(memory, offset)  (memory->node.lockAddress[gcvHARDWARE_3D].physical + offset + 63) & 0xFFFFFFFFFFFFFFC0
#endif

inline float HalfToFloat(uint16_t val)
{
    fi infnan;
    fi magic;
    fi f32;

    infnan.ui = 0x8f << 23;
    infnan.f = 65536.0f;
    magic.ui = 0xef << 23;

    /* Exponent / Mantissa */
    f32.ui = (val & 0x7fff) << 13;

    /* Adjust */
    f32.f *= magic.f;
    /* XXX: The magic mul relies on denorms being available */

    /* Inf / NaN */
    if (f32.f >= infnan.f)
        f32.ui |= 0xff << 23;

    /* Sign */
    f32.ui |= (uint32_t)(val & 0x8000) << 16;

    return f32.f;
}

inline int64_t u_intN_max(unsigned bit_size)
{
    return INT64_MAX >> (64 - bit_size);
}

inline float SnormToFloat(int x, unsigned src_bits)
{
    if (x <= -u_intN_max(src_bits))
        return -1.0f;
    else
        return x * (1.0f / (float)u_intN_max(src_bits));
}


//
// Bolt specific types and defines
//

#define BOLT_TRI_PER_MESH 2046

typedef VkAccelerationStructureInstanceKHR TlasEntryBolt;

typedef struct TriangleBolt {
    float                         vertices[3][3]; // 36 bytes
    uint32_t                      geometryID; // 40
    uint32_t                      primitiveID; // 44
    uint32_t                      meshID; // 48
    uint16_t                      type; // 50
    uint16_t                      flags; // 52
    uint64_t                      meshHdrAddr; //56
    uint32_t                      entryID; //=0x80000000
} TriangleBolt;

typedef struct NodeBolt {
    float                         nodes[14]; // 56 bytes
    uint32_t                      numEntries; // 60
    uint32_t                      entryID; // 
} NodeBolt;

typedef union BlasEntryBolt {
    NodeBolt                      Node;
    TriangleBolt                  Tri;
} BlasEntryBolt;

//
// BuildBLAS calls this for each vk geometry 
//
static uint32_t makeTriangles (
  const VkAccelerationStructureGeometryKHR* geom,
  const VkAccelerationStructureBuildRangeInfoKHR* range, 
  uint32_t geometry_id,
  uint8_t * vertices, 
  std::vector<TriangleBolt>& triangles,  
  uint32_t start_mesh_id )
{
  TriangleBolt Tri;
  NodeBolt Node;
  VkTransformMatrixKHR matrix;

  // get vk primitive count
  uint32_t primCnt = range->primitiveCount;

  // get memory address where index data is stored
  const VkAccelerationStructureGeometryTrianglesDataKHR* tri_data = &geom->geometry.triangles;

  // get index data
  uint8_t* index_data = (uint8_t*)((uint8_t*)tri_data->indexData.hostAddress + range->primitiveOffset);

  // set matrix to identity matrix
  __VK_MEMZERO((void *)&matrix, sizeof(VkTransformMatrixKHR));
  matrix.matrix[0][0] = 1.0f;
  matrix.matrix[1][1] = 1.0f;
  matrix.matrix[2][2] = 1.0f;

  // get memory address where transform matrix data is stored
  if (tri_data->transformData.hostAddress) {
    // get matrix if there is one
    matrix = *(const VkTransformMatrixKHR*)((const char*)tri_data->transformData.hostAddress + range->transformOffset);
  }

  // get the triangles from vertex and index buffers
  uint32_t index;
  uint32_t triCnt=0;
  uint32_t meshCnt=start_mesh_id;  
  for (uint32_t i=0; i<primCnt; i++) {    
    for (unsigned v=0; v<3; ++v) {
      index = range->firstVertex;
      switch (geom->geometry.triangles.indexType)
      {
        case VK_INDEX_TYPE_UINT8_EXT:
          index += *index_data;
          index_data += 1;
          break;

        case VK_INDEX_TYPE_UINT16:
          index += *(uint16_t*)index_data;
          index_data += 2;
          break;

        case VK_INDEX_TYPE_UINT32:
          index += *(uint32_t*)index_data;
          index_data += 4;
          break;

        case VK_INDEX_TYPE_NONE_KHR:
          index += i * 3 + v;
          break;

        default:
          break;
      }

      uint8_t * v_data = vertices + index * tri_data->vertexStride;
      float coords[4];

      switch (geom->geometry.triangles.vertexFormat)
      {
        case VK_FORMAT_R32G32_SFLOAT:
          coords[0] = *(float*)(v_data + 0);
          coords[1] = *(float*)(v_data + 4);
          coords[2] = 0.0f;
          coords[3] = 1.0f;
          break;
        case VK_FORMAT_R32G32B32_SFLOAT:
          coords[0] = *(float*)(v_data + 0);
          coords[1] = *(float*)(v_data + 4);
          coords[2] = *(float*)(v_data + 8);
          coords[3] = 1.0f;
          break;
        case VK_FORMAT_R32G32B32A32_SFLOAT:
          coords[0] = *(float*)(v_data + 0);
          coords[1] = *(float*)(v_data + 4);
          coords[2] = *(float*)(v_data + 8);
          coords[3] = *(float*)(v_data + 12);
          break;

        case VK_FORMAT_R16G16_SFLOAT:
          coords[0] = HalfToFloat(*(const uint16_t*)(v_data + 0));
          coords[1] = HalfToFloat(*(const uint16_t*)(v_data + 2));
          coords[2] = 0.0f;
          coords[3] = 1.0f;
          break;

        case VK_FORMAT_R16G16B16_SFLOAT:
          coords[0] = HalfToFloat(*(const uint16_t*)(v_data + 0));
          coords[1] = HalfToFloat(*(const uint16_t*)(v_data + 2));
          coords[2] = HalfToFloat(*(const uint16_t*)(v_data + 4));
          coords[3] = 1.0f;
          break;

        case VK_FORMAT_R16G16B16A16_SFLOAT:
          coords[0] = HalfToFloat(*(const uint16_t*)(v_data + 0));
          coords[1] = HalfToFloat(*(const uint16_t*)(v_data + 2));
          coords[2] = HalfToFloat(*(const uint16_t*)(v_data + 4));
          coords[3] = HalfToFloat(*(const uint16_t*)(v_data + 6));
          break;

        case VK_FORMAT_R16G16_SNORM:
          coords[0] = SnormToFloat(*(const int16_t*)(v_data + 0), 16);
          coords[1] = SnormToFloat(*(const int16_t*)(v_data + 2), 16);
          coords[2] = 0.0f;
          coords[3] = 1.0f;
          break;

        case VK_FORMAT_R16G16B16A16_SNORM:
          coords[0] = SnormToFloat(*(const int16_t*)(v_data + 0), 16);
          coords[1] = SnormToFloat(*(const int16_t*)(v_data + 2), 16);
          coords[2] = SnormToFloat(*(const int16_t*)(v_data + 4), 16);
          coords[3] = SnormToFloat(*(const int16_t*)(v_data + 6), 16);
          break;
        case VK_FORMAT_R16G16B16_SNORM:
          coords[0] = SnormToFloat(*(const int16_t*)(v_data + 0), 16);
          coords[1] = SnormToFloat(*(const int16_t*)(v_data + 2), 16);
          coords[2] = SnormToFloat(*(const int16_t*)(v_data + 4), 16);
          coords[3] = 1.0F;
          break;
        case VK_FORMAT_R8G8_SNORM:
          coords[0] = SnormToFloat(*(const int8_t*)(v_data + 0), 8);
          coords[1] = SnormToFloat(*(const int8_t*)(v_data + 1), 8);
          coords[2] = 0;
          coords[3] = 1.0F;
          break;
        case VK_FORMAT_R8G8B8_SNORM:
          coords[0] = SnormToFloat(*(const int8_t*)(v_data + 0), 8);
          coords[1] = SnormToFloat(*(const int8_t*)(v_data + 1), 8);
          coords[2] = SnormToFloat(*(const int8_t*)(v_data + 2), 8);
          coords[3] = 1.0F;
          break;
        case VK_FORMAT_R8G8B8A8_SNORM:
          coords[0] = SnormToFloat(*(const int8_t*)(v_data + 0), 8);
          coords[1] = SnormToFloat(*(const int8_t*)(v_data + 1), 8);
          coords[2] = SnormToFloat(*(const int8_t*)(v_data + 2), 8);
          coords[3] = SnormToFloat(*(const int8_t*)(v_data + 3), 8);
          break;
        default:
          break;
      }

      for (unsigned j = 0; j < 3; ++j) {
        float r = 0;
        for (unsigned k = 0; k < 4; ++k) {
          r += matrix.matrix[j][k] * coords[k];
        }
        // get the vertex transformed to AS space
        Tri.vertices[v][j] = r;
               
      }
    } // for (unsigned v=0; v<3; ++v) 

    Tri.geometryID = geometry_id; // vulkan geometry ID
    Tri.primitiveID = i; // vulkan primitive ID
    Tri.meshID = meshCnt; // bolt mesh ID
    Tri.type = geom->geometryType;
    Tri.flags = geom->flags;
    Tri.meshHdrAddr = 0;
    Tri.entryID = 0x80000000;

    triangles.push_back(Tri);
    
    // check if max tri-per-mesh exceeded
    triCnt++;
    if (triCnt==BOLT_TRI_PER_MESH) { 
      triCnt=0;
      meshCnt++;
    } 
  } //for (uint32_t i=0; i<primCnt; i++)

  // 
  return meshCnt;
}

uint32_t CalculateBLASSize(__vkDevContext* devCtx, uint32_t nTriangles)
{
  uint32_t memorySize;
  uint32_t numMeshes;

  //numMeshes = (nTriangles/BOLT_TRI_PER_MESH) + 1;
  memorySize = nTriangles * 0x40;
  //memorySize += numMeshes * 0x400;

  return memorySize; 
}

uint32_t CalculateTLASSize(__vkDevContext* devCtx, uint32_t nInstances)
{
  uint32_t memorySize;
 
  // TLAS node size
  memorySize = nInstances * 0x40;

  return memorySize;
}

// In this call, assemble the Vulkan geometries' triangles into Bolt AS meshes
//
// For BLAS, the following information is provided by Vulkan APIs:	
// - Number of geometries
// - Number of primitives (triangle list) and triangle list vertex data
// - Number of AABBS for geometry type VK_GEOMETRY_TYPE_AABBS_KHR
// - Source acceleration buffer address (for refit only) and destination acceleration buffer address.
// - A scratch buffer is used as a working buffer during acceleration structure building to store temporary results.
// - Some flags.

VkResult BuildBLAS(__vkDevContext* devCtx,
  const VkAccelerationStructureBuildGeometryInfoKHR* pInfos,
  const VkAccelerationStructureBuildRangeInfoKHR * pBuildRangeInfos)
{
  VkResult result = VK_SUCCESS;
  std::vector<TriangleBolt> triangles;
  uint64_t phyBLASAddr;
  uint8_t * blasPointer;
  uint32_t primitiveCount=0;
  uint32_t geoCount=0;  
  const VkAccelerationStructureGeometryKHR* geom = VK_NULL_HANDLE;
  bool succeed = true; // build successful

  // Get BLAS physical address
  __vkAccelerationStructure* blas = __VK_NON_DISPATCHABLE_HANDLE_CAST(__vkAccelerationStructure*, pInfos->dstAccelerationStructure);
  __vkBuffer* vb = __VK_NON_DISPATCHABLE_HANDLE_CAST(__vkBuffer*, blas->buffer); 
  phyBLASAddr = (uint64_t)((uint8_t*)blas->memory.hostAddr + blas->mem_offset + vb->memOffset + 63);
  phyBLASAddr &= 0xFFFFFFFFFFFFFFC0;
  blasPointer = (uint8_t*)phyBLASAddr;
  phyBLASAddr = GetDeviceAddress((&blas->memory), blas->mem_offset + vb->memOffset);

  // Get geometry and primitive count from all geometries
  geom = pInfos->pGeometries ? &pInfos->pGeometries[0] : pInfos->ppGeometries[0]; 
  for (geoCount = 0; geoCount < pInfos->geometryCount; geoCount++) {
    const VkAccelerationStructureBuildRangeInfoKHR* range = pBuildRangeInfos + geoCount;
    primitiveCount += range->primitiveCount;
  }

  // Reserve *based on primitive count
  triangles.reserve(primitiveCount);

  // Get triangles from all geometries 
  if (geom->geometryType == VK_GEOMETRY_TYPE_TRIANGLES_KHR) {
    uint32_t meshID = 0; // starting mesh ID
    for (geoCount = 0; geoCount < pInfos->geometryCount; geoCount++) {
      const VkAccelerationStructureBuildRangeInfoKHR * range = pBuildRangeInfos + geoCount;
      geom = pInfos->pGeometries ? (pInfos->pGeometries + geoCount) : pInfos->ppGeometries[geoCount];
      // get memory address where vertex data is stored
      uint64_t* vertices = (uint64_t*)geom->geometry.triangles.vertexData.hostAddress;
      meshID = makeTriangles(geom, range, geoCount, (uint8_t*)vertices, triangles, meshID);
    }

    // TBD: Input vertices into BVH builder if BVH to be done at host
    succeed = true;
  }
  else // #geometry_aabb
  {
    // No handler for aabbs
  }

  if (succeed) {
    // TBD: Copy only the triangles for now
    __VK_MEMCOPY(blasPointer, triangles, triangles.size()*sizeof(TriangleBolt));
  }
  else {
    __VK_MEMZERO(blasPointer, blas->size);
  }
  
  return result;
}


// In this call, create Bolt TLAS entries
//
// For TLAS, the following information is provided by Vulkan APIs:	
// - Number of instances.
// - Instance matrix
// - Source acceleration buffer address (for refit only) and destination acceleration buffer address.
// - A scratch buffer is used as a working buffer during acceleration structure building to store
//   temporary results.
// - Mask, flag, customIndex and instance shader binding table offset.

VkResult BuildTLAS(__vkDevContext* devCtx,
  const VkAccelerationStructureBuildGeometryInfoKHR* pInfos,
  const VkAccelerationStructureBuildRangeInfoKHR * pBuildRangeInfos)
{
  std::vector<TlasEntryBolt> tlasEntries;
  __vkAccelerationStructure* tlas = __VK_NON_DISPATCHABLE_HANDLE_CAST(__vkAccelerationStructure*, pInfos->dstAccelerationStructure);
  __vkAccelerationStructure* curblas = tlas;
  __vkBuffer* buf = VK_NULL_HANDLE;
  uint32_t geoCount, primCount;
  uint32_t instanceCount = 0;

  for (geoCount = 0; geoCount < pInfos->geometryCount; geoCount++) {
    const VkAccelerationStructureGeometryKHR* geom = pInfos->pGeometries ? &pInfos->pGeometries[geoCount] : pInfos->ppGeometries[geoCount];
    const VkAccelerationStructureBuildRangeInfoKHR* range = pBuildRangeInfos + geoCount;
    if (geom->geometryType == VK_GEOMETRY_TYPE_INSTANCES_KHR) {
      instanceCount += range->primitiveCount;
    }
  }

  // Reserved based on instanceCount
  tlasEntries.reserve(instanceCount);

  for (geoCount = 0; geoCount < pInfos->geometryCount; geoCount++) {
    const VkAccelerationStructureGeometryKHR* geom = pInfos->pGeometries ? &pInfos->pGeometries[geoCount] : pInfos->ppGeometries[geoCount];
    const VkAccelerationStructureBuildRangeInfoKHR* range = pBuildRangeInfos + geoCount;
    const VkAccelerationStructureGeometryInstancesDataKHR* inst_data = &geom->geometry.instances;
   
    // get memory address where matrix data is stored
    uint64_t* instancesData = (uint64_t*)geom->geometry.instances.data.hostAddress;

    if (geom->geometryType == VK_GEOMETRY_TYPE_INSTANCES_KHR) {
      for (primCount = 0; primCount < range->primitiveCount; primCount++) {
        const VkAccelerationStructureInstanceKHR* instance = inst_data->arrayOfPointers ? 
          (((const VkAccelerationStructureInstanceKHR* const*)instancesData)[primCount]) : 
          &((const VkAccelerationStructureInstanceKHR*)instancesData)[primCount];

        if (!instance->accelerationStructureReference) {
          continue;
        }

        // get address of the BLAS referenced by this instance
        __vkAccelerationStructure* blas = __VK_NON_DISPATCHABLE_HANDLE_CAST(__vkAccelerationStructure*, instance->accelerationStructureReference);
        __vkBuffer* vb = __VK_NON_DISPATCHABLE_HANDLE_CAST(__vkBuffer*, blas->buffer);
        uint8_t *blasPointer;
        uint64_t phyBLASAddr = (uint64_t)((uint8_t*)blas->memory.hostAddr + blas->mem_offset + vb->memOffset + 63);
        phyBLASAddr &= 0xFFFFFFFFFFFFFFC0;
        blasPointer = (uint8_t*)phyBLASAddr;
        uint64_t BLASMemory = GetDeviceAddress((&blas->memory), blas->mem_offset + vb->memOffset);

        // create Bolt TLAS entry, assign BLASMemory to accelerationStructureReference
        // assumes BLASMemory is accessible to RT IP in device
        TlasEntryBolt tlasEntry = *((TlasEntryBolt*)instance);
        tlasEntry.accelerationStructureReference = BLASMemory;
        tlasEntries.push_back(tlasEntry);
      }      
    }
  }

  // get TLAS memory virtual address where TLAS data will be stored
  uint8_t* tlasPointer;
  __vkBuffer* vb = __VK_NON_DISPATCHABLE_HANDLE_CAST(__vkBuffer*, tlas->buffer);
  uint64_t phyTLASAddr = (uint64_t)((uint8_t*)tlas->memory.hostAddr + tlas->mem_offset + vb->memOffset + 63);
  phyTLASAddr &= 0xFFFFFFFFFFFFFFC0;
  tlasPointer = (uint8_t *)phyTLASAddr;
  phyTLASAddr = GetDeviceAddress((&tlas->memory), tlas->mem_offset + vb->memOffset);

  __VK_MEMCOPY(tlasPointer, tlasEntries, tlasEntries.size() * sizeof(TlasEntryBolt));
  
  return VK_SUCCESS;
}
