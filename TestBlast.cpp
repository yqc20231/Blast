#define _CRT_SECURE_NO_WARNINGS
#include "NvBlastExtAuthoring.h"
#include "NvBlastExtAuthoringTypes.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <unordered_map>
#include <vector>

// PhysX 5.0
#include <PxPhysicsAPI.h>
// 包含Blast头文件（暂时不使用，但保留以验证路径）
#include "NvBlastExtAssetUtils.h"
#include "NvBlastExtAuthoring.h"
#include "NvBlastExtAuthoringBondGenerator.h"
#include "NvBlastExtAuthoringConvexMeshBuilder.h"
#include "NvBlastExtAuthoringFractureTool.h"
#include "NvBlastExtAuthoringMesh.h"
#include "NvBlastExtAuthoringMeshCleaner.h"
#include "NvBlastExtAuthoringPatternGenerator.h"
#include "NvBlastExtAuthoringTypes.h"
#include "NvBlastExtDamageShaders.h"
#include "NvBlastGlobals.h"
#include "NvBlastTk.h"
#include "NvBlastTkActor.h"
#include "NvBlastTkAsset.h"
#include "NvBlastTkFramework.h"
#include "nvblast.h"

using namespace physx;

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glfw3dll.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "NvBlast.lib")
#pragma comment(lib, "NvBlastTk.lib")
#pragma comment(lib, "NvBlastExtAuthoring.lib")
#pragma comment(lib, "NvBlastExtAssetUtils.lib")
#pragma comment(lib, "NvBlastGlobals.lib")
#pragma comment(lib, "NvBlastExtStress.lib")
#pragma comment(lib, "NvBlastExtShaders.lib")
#pragma comment(lib, "NvBlastGlobals.lib")

// PhysX 库
#pragma comment(lib, ".\\PhysX\\lib\\debug\\PhysXExtensions_static_64.lib")
#pragma comment(lib, ".\\PhysX\\lib\\debug\\PhysXCommon_64.lib")
#pragma comment(lib, ".\\PhysX\\lib\\debug\\PhysXFoundation_64.lib")
#pragma comment(lib, ".\\PhysX\\lib\\debug\\PhysXCooking_64.lib")

#pragma comment(lib, ".\\PhysX\\lib\\debug\\SnippetUtils_static_64.lib")
#pragma comment(lib, ".\\PhysX\\lib\\debug\\PVDRuntime_64.lib")
#pragma comment(lib, ".\\PhysX\\lib\\debug\\PhysX_64.lib")
#pragma comment(lib, ".\\PhysX\\lib\\debug\\PhysXFoundation_64.lib")
#pragma comment(lib, ".\\PhysX\\lib\\debug\\SceneQuery_static_64.lib")

// ======================= 全局对象 =======================
PxDefaultAllocator gAllocator;
PxDefaultErrorCallback gErrorCallback;
PxFoundation* gFoundation = nullptr;
PxPhysics* gPhysics = nullptr;
PxScene* gScene = nullptr;
PxMaterial* gMaterial = nullptr;
PxCookingParams* gCookingParams = nullptr;

// ========== 全局 Blast 对象 ==========
Nv::Blast::TkFramework* gTkFramework = nullptr;
NvBlastAsset* gLLAsset = nullptr; // 低级资产
Nv::Blast::TkAsset* gTkAsset = nullptr; // 高级资产
Nv::Blast::TkActor* gTkMainActor = nullptr; // 初始演员
Nv::Blast::TkFamily* gTkFamily = nullptr; // 家族
const NvBlastActor* gLLActor = nullptr; // 低级演员（用于施伤）


static PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0,
                                               PxFilterData filterData0,
                                               PxFilterObjectAttributes attributes1,
                                               PxFilterData filterData1,
                                               PxPairFlags& pairFlags,
                                               const void* constantBlock,
                                               PxU32 constantBlockSize)
{
    PX_UNUSED(attributes0);
    PX_UNUSED(attributes1);
    PX_UNUSED(filterData0);
    PX_UNUSED(filterData1);
    PX_UNUSED(constantBlockSize);
    PX_UNUSED(constantBlock);

  
    pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT | PxPairFlag::eNOTIFY_TOUCH_FOUND |
                PxPairFlag::eNOTIFY_TOUCH_LOST | PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eNOTIFY_CONTACT_POINTS;
    return PxFilterFlag::eDEFAULT;

    //
    // Enable CCD for the pair, request contact reports for initial and CCD contacts.
    // Additionally, provide information per contact point and provide the actor
    // pose at the time of contact.
   
    if (filterData0.word2 == 1 || 1 == filterData1.word2)
    {

        return PxFilterFlag::eKILL;
    }

    pairFlags = PxPairFlag::eSOLVE_CONTACT | PxPairFlag::eDETECT_DISCRETE_CONTACT | PxPairFlag::eNOTIFY_TOUCH_FOUND |
                PxPairFlag::eNOTIFY_TOUCH_LOST | PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eNOTIFY_CONTACT_POINTS;
    return PxFilterFlag::eDEFAULT;
}

// 物理数据
struct ChunkPhysX
{
    PxRigidDynamic* body = nullptr;
    PxConvexMesh* convexMesh = nullptr;
};
// 碎片物理映射
std::vector<ChunkPhysX> gChunkPhysX;
std::vector<PxFixedJoint*> gJoints; // 所有固定关节


void* BlastAlloc(size_t size)
{
    void* ptr = NvBlastGlobalGetAllocatorCallback()->allocate(size, nullptr, __FILE__, __LINE__);
    return ptr;
}

void BlastFree(void* ptr)
{
    NvBlastGlobalGetAllocatorCallback()->deallocate(ptr);
}

// PhysX 接触回调（用于触发伤害）
class MyContactCallback : public PxSimulationEventCallback
{
public:
    void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override
    {
        // 简化：检测是否有碎片与地面碰撞（假设地面是 static actor）
       
    }
    void onTrigger(PxTriggerPair* pairs, PxU32 count) override
    {
    }
    void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) override
    {
    }
    void onWake(PxActor** actors, PxU32 count) override
    {
    }
    void onSleep(PxActor** actors, PxU32 count) override
    {
    }
    void onAdvance(const PxRigidBody* const* bodyBuffer, const PxTransform* poseBuffer, const PxU32 count) override
    {
    }
};

// 渲染相关
struct ChunkRender
{
    GLuint vao, vbo, ebo;
    int indexCount;
    glm::vec3 centroid;
    uint32_t chunkId; // 对应的 Blast chunk ID
};
std::vector<ChunkRender> gChunks;
std::unordered_map<uint32_t, ChunkRender*> gChunkIdToRender; // chunkId -> render

// 简单随机数生成器
class SimpleRandom : public Nv::Blast::RandomGeneratorBase
{
public:
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist{ 0.0f, 1.0f };
    float getRandomValue() override
    {
        return dist(gen);
    }
    void seed(int32_t s) override
    {
        gen.seed(s);
    }
};
// ========== 辅助：空壳 ConvexMeshBuilder（仅用于创建 BondGenerator） ==========
class PxConvexMeshBuilder : public Nv::Blast::ConvexMeshBuilder
{
public:
    PxConvexMeshBuilder(const PxCookingParams& params) : mParams(params)
    {
    }

    virtual Nv::Blast::CollisionHull* buildCollisionGeometry(uint32_t verticesCount, const NvcVec3* vertexData) override
    {
        if (verticesCount < 4)
            return nullptr;

        PxConvexMeshDesc desc;
        desc.points.count = verticesCount;
        desc.points.stride = sizeof(NvcVec3);
        desc.points.data = vertexData;
        desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

        // 直接创建凸包（内部自动烹饪）
        PxConvexMesh* convex = PxCreateConvexMesh(mParams, desc);
        return reinterpret_cast<Nv::Blast::CollisionHull*>(convex);
    }

    virtual void releaseCollisionHull(Nv::Blast::CollisionHull* hull) const override
    {
        if (hull)
            reinterpret_cast<PxConvexMesh*>(hull)->release();
    }

    virtual void release() override
    {
        delete this;
    }

private:
    PxCookingParams mParams;
};

// 着色器源码
const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec3 Color;
void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    Color = normalize(aNormal) * 0.5 + 0.5;
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
in vec3 Color;
out vec4 FragColor;
void main() {
    FragColor = vec4(Color, 1.0);
}
)";

GLuint createShaderProgram()
{
    const char* vsSrc = R"(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 model, view, projection;
out vec3 Color;
void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    Color = normalize(aNormal) * 0.5 + 0.5;
})";
    const char* fsSrc = R"(
#version 330 core
in vec3 Color;
out vec4 FragColor;
void main() {
    FragColor = vec4(Color, 1.0);
})";
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(vs, 1, &vsSrc, nullptr);
    glCompileShader(vs);
    glShaderSource(fs, 1, &fsSrc, nullptr);
    glCompileShader(fs);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}


// 简单的 OBJ 加载器
bool loadOBJ(const std::string& path,
             std::vector<NvcVec3>& outPositions,
             std::vector<NvcVec3>& outNormals,
             std::vector<NvcVec2>& outUVs,
             std::vector<uint32_t>& outIndices)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Cannot open OBJ file: " << path << std::endl;
        return false;
    }

    std::vector<NvcVec3> tempPositions;
    std::vector<NvcVec3> tempNormals;
    std::vector<NvcVec2> tempUVs;

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        if (prefix == "v")
        {
            float x, y, z;
            iss >> x >> y >> z;
            tempPositions.push_back({ x, y, z });
        }
        else if (prefix == "vn")
        {
            float nx, ny, nz;
            iss >> nx >> ny >> nz;
            tempNormals.push_back({ nx, ny, nz });
        }
        else if (prefix == "vt")
        {
            float u, v;
            iss >> u >> v;
            tempUVs.push_back({ u, v });
        }
        else if (prefix == "f")
        {
            // 收集当前面的所有顶点（支持三角形和四边形）
            std::vector<std::string> vertices;
            std::string vertex;
            while (iss >> vertex)
            {
                vertices.push_back(vertex);
            }
            // 将四边形转换为两个三角形
            std::vector<std::string> triVerts;
            if (vertices.size() == 3)
            {
                triVerts = vertices;
            }
            else if (vertices.size() == 4)
            {
                triVerts = { vertices[0], vertices[1], vertices[2], vertices[0], vertices[2], vertices[3] };
            }
            else
            {
                std::cerr << "Unsupported face vertex count: " << vertices.size() << std::endl;
                continue;
            }
            for (auto& v : triVerts)
            {
                std::istringstream vss(v);
                std::string indexStr;
                int posIdx = -1, uvIdx = -1, nrmIdx = -1;
                // 解析 pos/uv/normal
                std::getline(vss, indexStr, '/');
                if (!indexStr.empty())
                    posIdx = std::stoi(indexStr);
                if (std::getline(vss, indexStr, '/'))
                {
                    if (!indexStr.empty())
                        uvIdx = std::stoi(indexStr);
                }
                if (std::getline(vss, indexStr, '/'))
                {
                    if (!indexStr.empty())
                        nrmIdx = std::stoi(indexStr);
                }
                if (posIdx == -1)
                    continue;

                // OBJ 索引从 1 开始
                int pos0 = posIdx - 1;
                // 检查是否需要新顶点（因为 Blast 需要每个位置对应唯一的法线和 UV）
                // 简化：直接按索引使用，但可能会产生重复顶点，导致开放边，不过 Blast 内部会处理
                outPositions.push_back(tempPositions[pos0]);
                if (nrmIdx >= 1)
                    outNormals.push_back(tempNormals[nrmIdx - 1]);
                else
                    outNormals.push_back({ 0, 0, 0 });
                if (uvIdx >= 1)
                    outUVs.push_back(tempUVs[uvIdx - 1]);
                else
                    outUVs.push_back({ 0, 0 });
                outIndices.push_back((uint32_t)(outPositions.size() - 1));
            }
        }
    }
    file.close();
    std::cout << "Loaded " << outPositions.size() << " vertices, " << outIndices.size() / 3 << " triangles." << std::endl;
    return true;
}

struct Chunk
{
    GLuint vao = 0, vbo = 0, ebo = 0;
    int indexCount = 0;
    glm::vec3 centroid{ 0.0f, 0.0f, 0.0f };
};

// ========== 刷新所有固定关节（根据当前 Bond 健康值） ==========
void refreshJoints()
{
    for (auto* j : gJoints)
        j->release();
    gJoints.clear();

    if (!gLLActor || !gLLAsset)
        return;
    printf("refreshJoints called, gLLActor=%p, gLLAsset=%p\n", gLLActor, gLLAsset);
    const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(gLLAsset, nullptr);
    uint32_t nodeCount = graph.nodeCount;
    if (nodeCount == 0)
        return;

    const float* bondHealths = NvBlastActorGetBondHealths(gLLActor, nullptr);
    uint32_t totalBonds = NvBlastAssetGetBondCount(gLLAsset, nullptr);
    std::vector<bool> processed(totalBonds, false); // 标记已处理的 bond

    for (uint32_t i = 0; i < nodeCount; ++i)
    {
        if (!gChunkPhysX[i].body)
            continue;


        uint32_t adjStart = graph.adjacencyPartition[i];
        uint32_t adjEnd = graph.adjacencyPartition[i + 1];
        for (uint32_t adj = adjStart; adj < adjEnd; ++adj)
        {
            uint32_t j = graph.adjacentNodeIndices[adj];
            uint32_t bondIdx = graph.adjacentBondIndices[adj];

            if (processed[bondIdx])
                continue; // 已经处理过

            static int debugJointCount = 0;
            if (debugJointCount < 5)
            {
                printf("Creating joint between node %u and %u, bondIdx %u, health %f\n", i, j, bondIdx,
                       bondHealths ? bondHealths[bondIdx] : -1.0f);
                debugJointCount++;
            }

            processed[bondIdx] = true;

            // 检查 bond 健康值
            if (bondHealths && bondHealths[bondIdx] <= 0.0f)
                continue;

            PxRigidDynamic* a0 = gChunkPhysX[i].body;
            PxRigidDynamic* a1 = gChunkPhysX[j].body;
            if (!a0 || !a1)
                continue;

            PxTransform rel = a1->getGlobalPose().transformInv(a0->getGlobalPose());
            PxFixedJoint* joint = PxFixedJointCreate(*gPhysics, a0, PxTransform(PxIdentity), a1, rel);
            joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
            gJoints.push_back(joint);
        }
    }
    printf("refreshJoints finished, created %u joints\n", (uint32_t)gJoints.size());
}

// ========== 分裂事件监听器 ==========
class BlastListener : public Nv::Blast::TkEventListener
{
public:
    void receive(const Nv::Blast::TkEvent* events, uint32_t count) override
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            if (events[i].type != Nv::Blast::TkEvent::Split)
                continue;
            auto* se = events[i].getPayload<Nv::Blast::TkSplitEvent>();
            for (uint32_t j = 0; j < se->numChildren; ++j)
            {
                Nv::Blast::TkActor* child = se->children[j];
                uint32_t visCnt = child->getVisibleChunkCount();
                std::vector<uint32_t> vis(visCnt);
                child->getVisibleChunkIndices(vis.data(), visCnt);
                const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(gLLAsset, nullptr);
                for (uint32_t c : vis)
                {
                    // 找到该 chunk 对应的支撑节点索引
                    int nodeIdx = -1;
                    for (uint32_t n = 0; n < graph.nodeCount; ++n)
                    {
                        if (graph.chunkIndices[n] == c)
                        {
                            nodeIdx = n;
                            break;
                        }
                    }
                    if (nodeIdx < 0 || gChunkPhysX[nodeIdx].body)
                        continue;

                    // 创建新刚体（使用已存储的凸包）
                    const NvBlastChunk* chunks = NvBlastAssetGetChunks(gLLAsset, nullptr);
                    PxVec3 centroid(chunks[c].centroid[0], chunks[c].centroid[1], chunks[c].centroid[2]);
                    PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(centroid));

                    if (gChunkPhysX[nodeIdx].convexMesh)
                    {
                        PxShape* shape =
                            gPhysics->createShape(PxConvexMeshGeometry(gChunkPhysX[nodeIdx].convexMesh), *gMaterial);
                        body->attachShape(*shape);
                        shape->release();
                    }
                    else
                    {
                        PxShape* shape = gPhysics->createShape(PxBoxGeometry(0.2f, 0.2f, 0.2f), *gMaterial);
                        body->attachShape(*shape);
                        shape->release();
                    }
                    body->setMass(1.0f);
                    gScene->addActor(*body);
                    gChunkPhysX[nodeIdx].body = body;
                }
            }
            refreshJoints();
        }
    }
};
// ---------- 从 FractureTool 创建 Blast 资产 ----------
NvBlastAsset* createAssetFromFractureTool(Nv::Blast::FractureTool* ftool, PxConvexMeshBuilder& builder)
{
    uint32_t chunkCount = ftool->getChunkCount();
    bool* isSupport = new bool[chunkCount];
    for (uint32_t i = 0; i < chunkCount; ++i)
    {
        isSupport[i] = false;
    }
    // std::vector<bool> isSupport(chunkCount, false);
    for (uint32_t i = 0; i < chunkCount; ++i)
        if (ftool->getChunkInfo(i).isLeaf)
            isSupport[i] = true;


    Nv::Blast::BlastBondGenerator* bondGen = NvBlastExtAuthoringCreateBondGenerator(&builder);
    NvBlastBondDesc* bondDescs = nullptr;
    NvBlastChunkDesc* chunkDescs = nullptr;
    int32_t bondCount = bondGen->buildDescFromInternalFracture(ftool, isSupport, bondDescs, chunkDescs);
    bondGen->release();

    delete[] isSupport;
    NvBlastAssetDesc desc;
    desc.chunkCount = chunkCount;
    desc.chunkDescs = chunkDescs;
    desc.bondCount = bondCount;
    desc.bondDescs = bondDescs;

    size_t memSize = NvBlastGetAssetMemorySize(&desc, nullptr);
    size_t scratchSize = NvBlastGetRequiredScratchForCreateAsset(&desc, nullptr);

    // 使用 BlastAlloc 替代 NvBlastGlobalAlloc
    void* mem = BlastAlloc(memSize);
    void* scratch = BlastAlloc(scratchSize);

    NvBlastAsset* asset = NvBlastCreateAsset(mem, &desc, scratch, nullptr);

    // 释放 scratch 内存（资产 mem 由 TkAsset 负责释放，因为 ownsAsset = true）
    BlastFree(scratch);

    NVBLAST_FREE(chunkDescs);
    NVBLAST_FREE(bondDescs);

    return asset;
}
struct BlastVertex
{
    float px, py, pz, nx, ny, nz, u, v;
};
NvBlastExtDamageAccelerator* gAccelerator = nullptr;

int main()
{


    NvBlastGlobalSetAllocatorCallback(nullptr);
    NvBlastGlobalSetErrorCallback(nullptr);


    // 初始化 GLFW 和 GLEW
    if (!glfwInit())
        return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    GLFWwindow* window = glfwCreateWindow(800, 600, "Blast OBJ Fracture - Eyeball", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glewInit();


    // 2. PhysX 初始化
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true);
    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0, -9.81f, 0);
    sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.filterShader = contactReportFilterShader;
    // PxDefaultSimulationFilterShader;

    gScene = gPhysics->createScene(sceneDesc);
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.1f);
    gCookingParams = new PxCookingParams(gPhysics->getTolerancesScale());
    gCookingParams->meshPreprocessParams = PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
    PxRigidStatic* ground = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 1), *gMaterial);
    gScene->addActor(*ground);

    // 3. Blast TkFramework
    gTkFramework = NvBlastTkFrameworkCreate();

    // 1. 加载 OBJ 模型（请将 eyeball.obj 放在可执行文件同目录）
    std::vector<NvcVec3> positions, normals;
    std::vector<NvcVec2> uvs;
    std::vector<uint32_t> indices;
    if (!loadOBJ("./eyeframe.obj", positions, normals, uvs, indices))
    {
        std::cerr << "未找到eyeframe.obj模型，请将OBJ文件放到程序同级目录\n";
        return -1;
    }
    
    //if (!loadOBJ("F:\\eyeframes\\eyeframe.obj", positions, normals, uvs, indices))
    //{
    //    std::cerr << "Failed to load OBJ, using fallback sphere." << std::endl;
    //    // 可选：这里可以生成一个球体代替
    //    return -1;
    //}
    // 2. 创建 Blast Mesh
    Nv::Blast::Mesh* mesh =
        NvBlastExtAuthoringCreateMesh(positions.data(), normals.data(), uvs.data(), (uint32_t)positions.size(),
                                      indices.data(), (uint32_t)indices.size());
    if (!mesh)
    {
        std::cerr << "Mesh creation failed" << std::endl;
        return -1;
    }

    // 3. 切片破碎（稳定、快速、碎片均匀）
    Nv::Blast::FractureTool* ftool = NvBlastExtAuthoringCreateFractureTool();
    Nv::Blast::Mesh* meshes[] = { mesh };
    ftool->setSourceMeshes(meshes, 1);

    SimpleRandom rng;
    rng.seed(42);
    Nv::Blast::SlicingConfiguration conf;
    conf.x_slices = 8; // 产生 4x4x4 = 64 块
    conf.y_slices = 8;
    conf.z_slices = 8;
    conf.angle_variations = 0.2f;
    conf.offset_variations = 0.2f;
    ftool->slicing(0, conf, false, &rng);
    ftool->finalizeFracturing();


    // 6. 提取叶子块渲染数据，并记录每个叶子块的顶点（用于凸包）
    Nv::Blast::Vertex* vb = nullptr;
    uint32_t *ib = nullptr, *ibOffsets = nullptr;
    ftool->getBufferedBaseMeshes(vb, ib, ibOffsets);
    uint32_t totalChunks = ftool->getChunkCount();

    std::vector<std::vector<NvcVec3>> leafVertices; // 每个叶子块的顶点，索引=chunkId
    leafVertices.resize(totalChunks);
    for (uint32_t i = 0; i < totalChunks; ++i)
    {
        if (!ftool->getChunkInfo(i).isLeaf)
            continue;
        uint32_t start = ibOffsets[i], end = ibOffsets[i + 1];
        uint32_t count = end - start;
        if (count == 0)
            continue;

        ChunkRender cr;
        cr.chunkId = ftool->getChunkId(i);
        std::vector<BlastVertex> verts(count);
        std::vector<unsigned int> indices(count);
        std::vector<NvcVec3> vtx(count);
        glm::vec3 centroid(0);
        for (uint32_t j = 0; j < count; ++j)
        {
            uint32_t vi = ib[start + j];
            const Nv::Blast::Vertex& v = vb[vi];
            verts[j] = { v.p.x, v.p.y, v.p.z, v.n.x, v.n.y, v.n.z, v.uv[0].x, v.uv[0].y };
            centroid += glm::vec3(v.p.x, v.p.y, v.p.z);
            vtx[j] = { v.p.x, v.p.y, v.p.z };
            indices[j] = j;
        }
        centroid /= (float)count;
        cr.centroid = centroid;

        glGenVertexArrays(1, &cr.vao);
        glGenBuffers(1, &cr.vbo);
        glGenBuffers(1, &cr.ebo);
        glBindVertexArray(cr.vao);
        glBindBuffer(GL_ARRAY_BUFFER, cr.vbo);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(BlastVertex), verts.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cr.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(BlastVertex), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(BlastVertex), (void*)offsetof(BlastVertex, nx));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        cr.indexCount = count;
        gChunks.push_back(cr);
        leafVertices[cr.chunkId] = std::move(vtx);
        printf("Support node %u (chunk %u) centroid: (%f, %f, %f)\n", i, cr.chunkId, cr.centroid[0], cr.centroid[1],
               cr.centroid[2]);
    }

    // 7. 创建真正的 ConvexMeshBuilder 并生成 Blast 资产
    PxConvexMeshBuilder* convexBuilder = new PxConvexMeshBuilder(*gCookingParams);
    gLLAsset = createAssetFromFractureTool(ftool, *convexBuilder);
    if (!gLLAsset)
    {
        std::cerr << "Asset creation failed" << std::endl;
        return -1;
    }

    // 8. TkAsset 和 TkActor
    gTkAsset = gTkFramework->createAsset(gLLAsset, nullptr, 0, true);
    gTkMainActor = gTkFramework->createActor(Nv::Blast::TkActorDesc(gTkAsset));
    gTkFamily = &gTkMainActor->getFamily();
    gLLActor = gTkMainActor->getActorLL();
    printf("gLLActor created: %p\n", gLLActor);
    // 9. 获取支撑图，为每个支撑碎片创建刚体
    const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(gLLAsset, nullptr);
    uint32_t supportCnt = graph.nodeCount;
    gChunkPhysX.resize(supportCnt);
    const NvBlastChunk* chunks = NvBlastAssetGetChunks(gLLAsset, nullptr);

    for (uint32_t s = 0; s < supportCnt; ++s)
    {
        uint32_t cid = graph.chunkIndices[s];
        if (cid >= totalChunks)
            continue;
        //const NvBlastChunk& chunk = chunks[cid];
        //PxVec3 centroid(chunk.centroid[0], chunk.centroid[1], chunk.centroid[2]);
        //PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(centroid));
        PxTransform initialPose(PxVec3(0, 0, 0), PxQuat(PxIdentity));
        PxRigidDynamic* body = gPhysics->createRigidDynamic(initialPose);

        // 保留运动学、阻尼配置
        //body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
        body->setAngularDamping(10.0f);
        body->setLinearDamping(5.0f);
        body->setMaxAngularVelocity(1.0f);

        // 从对应的叶子块获取顶点生成凸包
        PxConvexMesh* convex = nullptr;
        if (!leafVertices[cid].empty())
        {
            auto& vtx = leafVertices[cid];
            convex =
                reinterpret_cast<PxConvexMesh*>(convexBuilder->buildCollisionGeometry((uint32_t)vtx.size(), vtx.data()));
        }
        if (convex)
        {
            PxShape* shape = gPhysics->createShape(PxConvexMeshGeometry(convex), *gMaterial);
            body->attachShape(*shape);
            shape->release();
        }
        else
        {
            PxShape* shape = gPhysics->createShape(PxBoxGeometry(0.2f, 0.2f, 0.2f), *gMaterial);
            body->attachShape(*shape);
            shape->release();
        }
        body->setMass(1.0f);
        gScene->addActor(*body);
        gChunkPhysX[s].body = body;
        gChunkPhysX[s].convexMesh = convex;
    }
    // 10. 创建初始关节，注册分裂监听器
    refreshJoints();
    BlastListener listener;
    gTkFamily->addListener(listener);

    // 释放临时资源
    ftool->release();
    mesh->release();


    // 11. 渲染循环
    GLuint shader = createShaderProgram();
    glUseProgram(shader);
    GLint modelLoc = glGetUniformLocation(shader, "model");
    GLint viewLoc = glGetUniformLocation(shader, "view");
    GLint projLoc = glGetUniformLocation(shader, "projection");
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
    glm::mat4 view = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -3.0f));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(proj));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.2f, 0.2f, 0.3f, 1.0f);

    float dt = 1.0f / 60.0f;
    while (!glfwWindowShouldClose(window))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 空格键触发伤害
        static bool prevSpace = false;
        bool space = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        if (space && !prevSpace)
        {
            // ==========================
            // 1. 随机生成爆炸中心点（模型内部随机位置）
            // ==========================
            PxVec3 explosionCenter((rand() % 100 - 50) / 80.0f, // X 随机
                                   (rand() % 100 - 50) / 80.0f, // Y 随机
                                   (rand() % 100 - 50) / 80.0f // Z 随机
            );


            NvBlastExtImpactSpreadDamageDesc dmgDesc;
            dmgDesc.position[0] = explosionCenter.x;
            dmgDesc.position[1] = explosionCenter.y;
            dmgDesc.position[2] = explosionCenter.z; // 伤害中心点
            dmgDesc.damage = 0.8f; // 伤害强度（0~1）
            dmgDesc.minRadius = 0.05f; // 最小半径
            dmgDesc.maxRadius = 0.5f; // 最大扩散半径

            gAccelerator = NvBlastExtDamageAcceleratorCreate(gLLAsset, 1);

            if (gAccelerator)
            {
                // 2. 填充参数
                NvBlastExtProgramParams params(&dmgDesc, nullptr, gAccelerator);

                // 准备命令缓冲区
                uint32_t chunkCount = NvBlastAssetGetChunkCount(gLLAsset, nullptr);
                uint32_t bondCount = NvBlastAssetGetBondCount(gLLAsset, nullptr);
                std::vector<NvBlastChunkFractureData> chunkCommands(chunkCount);
                std::vector<NvBlastBondFractureData> bondCommands(bondCount);
                NvBlastFractureBuffers cmdBuffers;
                cmdBuffers.chunkFractureCount = chunkCount;
                cmdBuffers.chunkFractures = chunkCommands.data();
                cmdBuffers.bondFractureCount = bondCount;
                cmdBuffers.bondFractures = bondCommands.data();

                // 准备事件缓冲区（可复用同一个内存）
                std::vector<NvBlastChunkFractureData> chunkEvents(chunkCount);
                std::vector<NvBlastBondFractureData> bondEvents(bondCount);
                NvBlastFractureBuffers evtBuffers;
                evtBuffers.chunkFractureCount = chunkCount;
                evtBuffers.chunkFractures = chunkEvents.data();
                evtBuffers.bondFractureCount = bondCount;
                evtBuffers.bondFractures = bondEvents.data();

                // 构造损伤程序
                NvBlastDamageProgram program;
                program.graphShaderFunction = NvBlastExtImpactSpreadGraphShader;
                program.subgraphShaderFunction = NvBlastExtImpactSpreadSubgraphShader;

                // 1. 生成骨折命令
                NvBlastActorGenerateFracture(&cmdBuffers, gLLActor, program, &params, nullptr, nullptr);
                // 2. 应用骨折
                NvBlastActorApplyFracture(
                    &evtBuffers, const_cast<NvBlastActor*>(gLLActor), &cmdBuffers, nullptr, nullptr);
                // 3. 分裂 Actor
                if (NvBlastActorIsSplitRequired(gLLActor, nullptr))
                {
                    uint32_t maxActorCount = NvBlastActorGetMaxActorCountForSplit(gLLActor, nullptr);
                    std::vector<NvBlastActor*> newActorBuffer(maxActorCount, nullptr);
                    NvBlastActorSplitEvent splitEvent;
                    splitEvent.deletedActor = nullptr; // 可能被设置，但我们不关心
                    splitEvent.newActors = newActorBuffer.data();

                    size_t scratchSize = NvBlastActorGetRequiredScratchForSplit(gLLActor, nullptr);
                    void* scratch = _aligned_malloc(scratchSize, 16);
                    uint32_t newActors = NvBlastActorSplit(
                        &splitEvent, const_cast<NvBlastActor*>(gLLActor), maxActorCount, scratch, nullptr, nullptr);
                    _aligned_free(scratch);


                    // 为新生成的 Actor 创建物理刚体
                    if (newActors > 0)
                    {
                        const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(gLLAsset, nullptr);
                        const NvBlastChunk* chunks = NvBlastAssetGetChunks(gLLAsset, nullptr);

                        for (uint32_t i = 0; i < newActors; ++i)
                        {
                            NvBlastActor* newActorLL = splitEvent.newActors[i];
                            if (!newActorLL)
                                continue;

                            uint32_t visCnt = NvBlastActorGetVisibleChunkCount(newActorLL, nullptr);
                            std::vector<uint32_t> visChunks(visCnt);
                            NvBlastActorGetVisibleChunkIndices(visChunks.data(), visCnt, newActorLL, nullptr);

                            for (uint32_t v = 0; v < visCnt; ++v)
                            {
                                uint32_t chunkId = visChunks[v];
                                // 查找对应的支撑节点索引
                                int nodeIdx = -1;
                                for (uint32_t n = 0; n < graph.nodeCount; ++n)
                                {
                                    if (graph.chunkIndices[n] == chunkId)
                                    {
                                        nodeIdx = (int)n;
                                        break;
                                    }
                                }
                                if (nodeIdx < 0)
                                    continue;

                                // 如果该支撑块尚未创建刚体，则创建
                                if (gChunkPhysX[nodeIdx].body == nullptr)
                                {
                                    PxVec3 centroid(chunks[chunkId].centroid[0], chunks[chunkId].centroid[1],
                                                    chunks[chunkId].centroid[2]);
                                    PxRigidDynamic* body = gPhysics->createRigidDynamic(PxTransform(centroid));

                                    if (gChunkPhysX[nodeIdx].convexMesh)
                                    {
                                        PxShape* shape = gPhysics->createShape(
                                            PxConvexMeshGeometry(gChunkPhysX[nodeIdx].convexMesh), *gMaterial);
                                        body->attachShape(*shape);
                                        shape->release();
                                    }
                                    else
                                    {
                                        PxShape* shape =
                                            gPhysics->createShape(PxBoxGeometry(0.2f, 0.2f, 0.2f), *gMaterial);
                                        body->attachShape(*shape);
                                        shape->release();
                                    }

                                    body->setMass(1.0f);
                                    gScene->addActor(*body);
                                    gChunkPhysX[nodeIdx].body = body;
                                }
                                // ==========================
                                // ✅ 关键：给刚碎的块加冲击力
                                // ==========================
                                PxRigidDynamic* body = gChunkPhysX[nodeIdx].body;
                                if (body)
                                {
                                    body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
                                    body->wakeUp();

                                    PxVec3 pos = body->getGlobalPose().p;
                                    PxVec3 dir = pos - explosionCenter;
                                    float dist = dir.magnitude();

                                    if (dist > 0.01f)
                                        dir.normalize();
                                    else
                                        dir = PxVec3(0, 1, 0);

                                    float radius = 1.2f;
                                    float attn = 1.0f - (dist / radius);
                                    if (attn < 0)
                                        attn = 0;

                                    // 施加冲量
                                    body->addForce(dir * 1000.0f * attn, PxForceMode::eIMPULSE);

                                    // 随机旋转，更酥散
                                    PxVec3 torque((rand() % 100 - 50) / 15.0f, (rand() % 100 - 50) / 15.0f,
                                                  (rand() % 100 - 50) / 15.0f);
                                    body->addTorque(torque * 180.0f, PxForceMode::eIMPULSE);
                                }
                            }
                        }
                    }
                }
            }
            
            refreshJoints(); // 确保关节状态同步
        }
        prevSpace = space;

        gScene->simulate(dt);
        gScene->fetchResults(true);

        // 绘制所有叶子碎片，跟随其支撑块刚体的变换
        for (auto& cr : gChunks)
        {
            int sIdx = -1;
            for (uint32_t s = 0; s < supportCnt; ++s)
            {
                if (graph.chunkIndices[s] == cr.chunkId)
                {
                    sIdx = s;
                    break;
                }
            }
            if (sIdx < 0 || !gChunkPhysX[sIdx].body)
                continue;
            PxTransform pose = gChunkPhysX[sIdx].body->getGlobalPose();
            glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(pose.p.x, pose.p.y, pose.p.z));
            model = model * glm::mat4_cast(glm::quat(pose.q.w, pose.q.x, pose.q.y, pose.q.z));
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            glBindVertexArray(cr.vao);
            glDrawElements(GL_TRIANGLES, cr.indexCount, GL_UNSIGNED_INT, 0);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 12. 清理
    for (auto* j : gJoints)
        j->release();
    for (auto& cp : gChunkPhysX)
    {
        if (cp.body)
            cp.body->release();
        if (cp.convexMesh)
            cp.convexMesh->release();
    }
    gTkFamily->removeListener(listener);
    gTkMainActor->release();
    gTkAsset->release();
    gTkFramework->release();
    convexBuilder->release();
    delete gCookingParams;
    gScene->release();
    gPhysics->release();
    gFoundation->release();
    for (auto& cr : gChunks)
    {
        glDeleteVertexArrays(1, &cr.vao);
        glDeleteBuffers(1, &cr.vbo);
        glDeleteBuffers(1, &cr.ebo);
    }
    glDeleteProgram(shader);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
