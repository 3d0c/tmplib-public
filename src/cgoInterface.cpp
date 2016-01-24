// #include "libdef.h"
#include "libdress.h"
#include "DFCache.h"
#include <stdio.h>
#include <string>
#include <sstream>
#include "DFFitting.h"

using namespace std;

struct FittingStruct {
    DFFitting fitting;
    DF::PointsVector points;
    DF::FacesVector faces;
    DF::faceItemVector face;
    DF::FaceItem face_item;
    DFMassSpringObject mesh;
    Log log;
    time_t start_func;
    time_t stop_func;
    int sessionID;

    FittingStruct(DFMannequin::MannequinStorage &storage) {
        sessionID = rand();

        log << "[" << DF::GetCurrentTimeString() << "] Function has just started (session ID = " << std::hex << sessionID << std::dec << ")\n";

        start_func = clock();

        mesh.SetPrecession(4);
        fitting.SetMannequinStorage(storage);
    }

    ~FittingStruct() {
        fitting.Release();
        DF::ReleaseSTLContainer(points);
        DF::ReleaseSTLContainer(faces);
        DF::ReleaseSTLContainer(face);
        mesh.Release();
        stop_func = clock();
        log << "[" << DF::GetCurrentTimeString() << "] Function is finishing (" << (static_cast<double>(stop_func - start_func) / CLOCKS_PER_SEC) << " sec)(session ID = " << std::hex << sessionID << std::dec << ")\n\n";
    }
};

const DF::FloatingPointType lenght_translating_rate = static_cast<DF::FloatingPointType>(0.1);

const static double time_limit_first_frame = 2.5;
const static double time_limit_next_frame = 0.5;
const static double time_limit_total = 10.;

void *NewDummyStorage() {
    return (void *)new (DFMannequin::MannequinStorage);
}

void ReleaseDummyStorage(void *dummy_storage, char *dummy_name) {
    DFMannequin::MannequinStorage &storage = *static_cast<DFMannequin::MannequinStorage *>(dummy_storage);

    string dummyName(dummy_name);
    storage[dummyName].Release();
}

int DummyAddMorphTarget(void *dummy_storage, char *dummy_name, char *file_name, char *param_name, double paramValue) {
    string dummyName(dummy_name);

    DFMannequin::MannequinStorage &storage = *static_cast<DFMannequin::MannequinStorage *>(dummy_storage);

    string fileName(file_name);
    string paramName(param_name);

    if (storage[dummyName].AddMorphTarget(fileName, paramName, paramValue * lenght_translating_rate) != DF::err_ok) {
        fprintf(stderr, "Unknown error\n");
        return -1;
    }

    fprintf(stderr, "MT Loaded: %s, %s, %.4f\n", fileName.c_str(), paramName.c_str(), paramValue);

    return 0;
}

int DummyMorph(void *dummy_storage, char *dummy_name, char *dst, morph_param **morph_params, int p_len) {
    DFMannequin::MorphParams params;

    DFMannequin::MannequinStorage &storage = *static_cast<DFMannequin::MannequinStorage *>(dummy_storage);

    FittingStruct fittingStruct(storage);

    DF::Error_t err;
    Log log;

    fprintf(stderr, "dummy_name: %s, dst: %s, p_len: %d\n", dummy_name, dst, p_len);

    string dummyName(dummy_name);
    string dstFile(dst);

    for (int i = 0; i < p_len; i++) {
        string pName(morph_params[i]->name);
        params[pName] = morph_params[i]->value * lenght_translating_rate;

        fprintf(stderr, "Name: %s, val: %.4f\n", pName.c_str(), params[pName]);
    }
    
    if (storage[dummyName].Morph(fittingStruct.fitting.GetFittingElement(0), params, false) != DF::err_ok) {
        fprintf(stderr, "Dummy morphing error\n");
        return -1;
    }

    if (dstFile.empty()) {
        return 0;
    }

    if ((err = fittingStruct.fitting.GetFittingElement(0).ExportOBJ(dstFile, OUT_VERTICES | OUT_TEXTURES | OUT_NORMALS | OUT_FACES, false, 0)) != DF::err_ok) {
        log << "Unable to ExportOBJ, error: " << DF::print_error(err) << ")\n";
        return -1;
    }

    return 0;
}

int DummyPutOn(void *dummy_storage, char *dummy_name, morph_param **morph_params, int p_len, garment **garments, int g_len) {
    DFMannequin::MannequinStorage &storage = *static_cast<DFMannequin::MannequinStorage *>(dummy_storage);

    FittingStruct fittingStruct(storage);
    DFMannequin::MorphParamsArray cachedParams(0);

    string dummyName(dummy_name);

    DF::Error_t err;
    Log log;

    DFMannequin::MorphParams params;

    for (int i = 0; i < p_len; i++) {
        string pName(morph_params[i]->name);
        params[pName] = morph_params[i]->value * lenght_translating_rate;

        fprintf(stderr, "Name: %s, val: %.4f\n", pName.c_str(), params[pName]);
    }

    for (int i = 0; i < g_len; i++) {
        string filename(garments[i]->input);

        if (fittingStruct.fitting.AddDress(filename) != DF::err_ok) {
            log << "Unable to AddDress" << filename << "\n";
            return -1;
        }
    }

    try {
        if (fittingStruct.fitting.FitAnimated(
                dummyName,
                params,
                cachedParams,
                "",
                40,
                0,
                time_limit_first_frame,
                time_limit_next_frame,
                time_limit_total) != DF::err_ok) {
            log << "FitAnimated failed\n";
            return -1;
        }
    }
    catch (...) {
        log << "Oops, exception catched\n";
        return -1;
    }

    for (int i = 0; i < g_len; i++) {
        DFMassSpringObject &dress = fittingStruct.fitting.GetFittingElement(i + 1);

        dress.SmoothMesh(DFMesh::SMOOTH_POST, static_cast<DF::FloatingPointType>(1));
        dress.SolveCollizion(
            fittingStruct.fitting.GetFittingDeque().cbegin(),
            fittingStruct.fitting.GetFittingDeque().cbegin() + i + 1,
            dress.physical_params()[0].shift());

        string filename(garments[i]->output);

        if ((err = dress.ExportOBJ(filename, OUT_VERTICES | OUT_TEXTURES | OUT_NORMALS | OUT_FACES, false, 0)) != DF::err_ok) {
            log << "Unable to ExportOBJ, error: " << DF::print_error(err) << ")\n";
            return -1;
        }
    }

    return 0;
}
