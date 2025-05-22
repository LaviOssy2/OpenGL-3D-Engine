#include "model/obj_loader.h"

bool loadOBJ(const std::string&    path,
             AAssetManager*        mgr,
             std::vector<Vertex>&       outV,
             std::vector<unsigned int>& outI)
{
    outV.clear();
    outI.clear();

    AAsset* asset = AAssetManager_open(mgr, path.c_str(), AASSET_MODE_BUFFER);
    if (!asset) {
        __android_log_print(ANDROID_LOG_ERROR, "OBJ", "cannot open %s", path.c_str());
        return false;
    }
    size_t len = AAsset_getLength(asset);
    std::string data(len, '\0');
    AAsset_read(asset, &data[0], len);
    AAsset_close(asset);

    std::istringstream file(data);
    std::vector<glm::vec3> pos;
    std::vector<glm::vec2> uv;
    std::vector<glm::vec3> nm;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string tag; ss >> tag;
        if (tag == "v") {
            glm::vec3 p; ss >> p.x >> p.y >> p.z;
            pos.push_back(p);
        }
        else if (tag == "vt") {
            glm::vec2 t; ss >> t.x >> t.y;
            uv.push_back(t);
        }
        else if (tag == "vn") {
            glm::vec3 n; ss >> n.x >> n.y >> n.z;
            nm.push_back(n);
        }
        else if (tag == "f") {
            std::vector<Vertex> faceVerts;
            std::string tok;

            while (ss >> tok) {
                unsigned int vi = 0, ti = 0, ni = 0;
                sscanf(tok.c_str(), "%u/%u/%u", &vi, &ti, &ni);
                Vertex vtx;
                vtx.position = pos[vi - 1];
                vtx.texCoord = uv[ti - 1];
                vtx.normal   = nm[ni - 1];
                faceVerts.push_back(vtx);
            }

            // Triangulate polygon (fan method)
            for (size_t i = 1; i + 1 < faceVerts.size(); ++i) {
                outV.push_back(faceVerts[0]);
                outV.push_back(faceVerts[i]);
                outV.push_back(faceVerts[i + 1]);

                outI.push_back((unsigned int)outV.size() - 3);
                outI.push_back((unsigned int)outV.size() - 2);
                outI.push_back((unsigned int)outV.size() - 1);
            }
        }

    }

    return true;
}
