#ifndef _cgoif_h_
#define _cgoif_h_

#include <sys/time.h>
#include <stdlib.h>

typedef unsigned long DWORD;

typedef struct {
    char *name;
    double value;
} morph_param;

typedef struct {
    char *input;
    char *output;
} garment;

#ifdef __cplusplus
extern "C" {
#endif

void *NewDummyStorage();
void ReleaseDummyStorage(void *dummy_storage, char *dummy_name);

void *NewFittingStruct(void *dummy_storage);

int DummyAddMorphTarget(void *dummy_storage, char *dummy_name, char *file_name, char *param_name, double paramValue);

int DummyMorph(void *dummy_storage, char *dummy_name, char *dst, morph_param **morph_params, int p_len);

int DummyPutOn(void *dummy_storage, char *dummy_name, morph_param **morph_params, int p_len, garment **garments, int g_len);

#ifdef __cplusplus
}
#endif

#endif
