#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <getopt.h>
#include <audiofile.h>
#include "tabledesign.h"

#ifdef __sgi

typedef long SampleFormat;

#define MODE_READ "r"

#else

// The modern implementation of SGI's audiofile library which is in Ubuntu
// (https://github.com/mpruett/audiofile/) has renamed some of the functions,
// and changed some data types.

typedef int SampleFormat;
#define AFopenfile afOpenFile
#define AFgetchannels afGetChannels
#define AFgettrackids afGetTrackIDs
#define AFgetsampfmt afGetSampleFormat
#define AFgetframecnt afGetFrameCount
#define AFgetrate afGetRate
#define AFreadframes afReadFrames

#define MODE_READ "rb"

#endif

char usage[80] = "[-o order -s bits -t thresh -i refine_iter -f frame_size] aifcfile";

int main(int argc, char **argv)
{
    const char *programName; // sp118
    double thresh; // sp110
    int order; // sp10C
    int bits; // sp108
    int refineIters; // sp104
    int frameSize; // sp100
    UNUSED int rate;
    int frameCount;
    int opt;
    double *spF4;
    double dummy; // spE8
    double **mat; // spE4
    double **data; // spD0
    double *splitDelta; // spCC
    int j; // spC0
    int permDet;
    int curBits; // spB8
    int npredictors; // spB4
    int *perm; // spB0
    int numOverflows; // spAC
    SampleFormat sampleFormat; // sp90
    SampleFormat sampleWidth; // sp8C
    AFfilehandle afFile; // sp88
    int channels;
    int tracks;
    double *vec; // s2
    double **temp_s1;
    short *temp_s3;
    int i;
    int dataSize; // s4

    order = 2;
    bits = 2;
    refineIters = 2;
    frameSize = 16;
    numOverflows = 0;
    programName = argv[0];
    thresh = 10.0;

    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] Enter tabledesign.c\n");

    if (argc < 2)
    {
        fprintf(stderr, "%s %s\n", argv[0], usage);
        exit(1);
    }

    while ((opt = getopt(argc, argv, "o:s:t:i:f:")) != -1)
    {
        switch (opt)
        {
        case 'o':
            if (sscanf(optarg, "%d", &order) != 1)
                order = 2;
            break;
        case 's':
            if (sscanf(optarg, "%d", &bits) != 1)
                bits = 2;
            break;
        case 'f':
            if (sscanf(optarg, "%d", &frameSize) != 1)
                frameSize = 16;
            break;
        case 'i':
            if (sscanf(optarg, "%d", &refineIters) != 1)
                refineIters = 2;
            break;
        case 't':
            if (sscanf(optarg, "%lf", &thresh) != 1)
                thresh = 10.0;
            break;
        }
    }

    argv = &argv[optind - 1];

    fprintf(stderr, "\n--- [tools/sdk-tools/tabledesign.c] ---\n");
    fprintf(stderr, "ABOUT TO CALL AFopenfile\n");
    fprintf(stderr, "argv[1] = %s\n", argv[1]);
    fprintf(stderr, "MODE_READ = %s\n", MODE_READ);
    fprintf(stderr, "----------------------------------------\n\n");
    afFile = AFopenfile(argv[1], MODE_READ, NULL);

    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] CALLED AFopenfile (i.e. afOpenFile)\n");
    if (afFile == NULL)
    {
        fprintf(stderr,
                "%s: input AIFC file [%s] could not be opened.\n",
                programName, argv[1]);
        fprintf(stderr, "[tools/sdk-tools/tabledesign.c] exit(1) - AFopen Error\n");
        exit(1);
    }

    channels = AFgetchannels(afFile, AF_DEFAULT_TRACK);
    if (channels != 1)
    {
        fprintf(stderr,
                "%s: file [%s] contains %d channels, only 1 channel supported.\n",
                programName, argv[1], channels);
        exit(1);
    }

    tracks = AFgettrackids(afFile, NULL);
    if (tracks != 1)
    {
        fprintf(stderr,
                "%s: file [%s] contains %d tracks, only 1 track supported.\n",
                programName, argv[1], tracks);
        exit(1);
    }

    AFgetsampfmt(afFile, AF_DEFAULT_TRACK, &sampleFormat, &sampleWidth);
    if (sampleWidth != 16)
    {
        fprintf(stderr,
                "%s: file [%s] contains %d bit samples, only 16 bit samples supported.\n",
                programName, argv[1], (int)sampleWidth);
        exit(1);
    }

    temp_s1 = malloc((1 << bits) * sizeof(double*));
    for (i = 0; i < (1 << bits); i++)
    {
        temp_s1[i] = malloc((order + 1) * sizeof(double));
    }

    splitDelta = malloc((order + 1) * sizeof(double));
    temp_s3 = malloc(frameSize * 2 * sizeof(short));
    for (i = 0; i < frameSize * 2; i++)
    {
        temp_s3[i] = 0;
    }

    vec = malloc((order + 1) * sizeof(double));
    spF4 = malloc((order + 1) * sizeof(double));
    mat = malloc((order + 1) * sizeof(double*));
    for (i = 0; i <= order; i++)
    {
        mat[i] = malloc((order + 1) * sizeof(double));
    }

    perm = malloc((order + 1) * sizeof(int));
    frameCount = AFgetframecnt(afFile, AF_DEFAULT_TRACK);
    rate = AFgetrate(afFile, AF_DEFAULT_TRACK);
    data = malloc(frameCount * sizeof(double*));
    dataSize = 0;

    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] opt = %d\n", opt);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] rate = %d\n", rate);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] channels = %d\n", channels);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] tracks = %d\n", tracks);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] frameCount = %d\n", frameCount);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] About to start afReadFrames AFreadframes loop\n");
    int debug_num_iters_l0 = 0;
    int debug_num_iters_l1 = 0;
    int debug_num_iters_l2 = 0;
    while (AFreadframes(afFile, AF_DEFAULT_TRACK, temp_s3 + frameSize, frameSize) == frameSize)
    {
        acvect(temp_s3 + frameSize, order, frameSize, vec);
        if (fabs(vec[0]) > thresh)
        {
            acmat(temp_s3 + frameSize, order, frameSize, mat);
            if (lud(mat, order, perm, &permDet) == 0)
            {
                lubksb(mat, order, perm, vec);
                vec[0] = 1.0;
                if (kfroma(vec, spF4, order) == 0)
                {
                    data[dataSize] = malloc((order + 1) * sizeof(double));
                    data[dataSize][0] = 1.0;

                    for (i = 1; i <= order; i++)
                    {
                        if (spF4[i] >=  1.0) spF4[i] =  0.9999999999;
                        if (spF4[i] <= -1.0) spF4[i] = -0.9999999999;
                        debug_num_iters_l1 += 1;
                    }

                    afromk(spF4, data[dataSize], order);
                    dataSize++;
                }
            }
        }

        for (i = 0; i < frameSize; i++)
        {
            temp_s3[i] = temp_s3[i + frameSize];
            debug_num_iters_l2 += 1;
        }
        debug_num_iters_l0 += 1;
        fprintf(stderr, "\r[tools/sdk-tools/tabledesign.c] status: %d, %d, %d, %d", dataSize, debug_num_iters_l0, debug_num_iters_l1, debug_num_iters_l2);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] End loop part\n");
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] dataSize: %d\n", dataSize);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] Num outer iterations %d\n", debug_num_iters_l0);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] Num inner iterations1 %d\n", debug_num_iters_l1);
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] Num inner iterations2 %d\n", debug_num_iters_l2);

    vec[0] = 1.0;
    for (j = 1; j <= order; j++)
    {
        vec[j] = 0.0;
    }

    for (i = 0; i < dataSize; i++)
    {
        rfroma(data[i], order, temp_s1[0]);
        for (j = 1; j <= order; j++)
        {
            vec[j] += temp_s1[0][j];
        }
    }

    for (j = 1; j <= order; j++)
    {
        vec[j] /= dataSize;
    }

    durbin(vec, order, spF4, temp_s1[0], &dummy);

    for (j = 1; j <= order; j++)
    {
        if (spF4[j] >=  1.0) spF4[j] =  0.9999999999;
        if (spF4[j] <= -1.0) spF4[j] = -0.9999999999;
    }

    afromk(spF4, temp_s1[0], order);
    curBits = 0;
    while (curBits < bits)
    {
        for (i = 0; i <= order; i++)
        {
            splitDelta[i] = 0.0;
        }
        splitDelta[order - 1] = -1.0;
        split(temp_s1, splitDelta, order, 1 << curBits, 0.01);
        curBits++;
        refine(temp_s1, order, 1 << curBits, data, dataSize, refineIters, 0.0);
    }

    npredictors = 1 << curBits;
    fprintf(stdout, "%d\n%d\n", order, npredictors);

    for (i = 0; i < npredictors; i++)
    {
        numOverflows += print_entry(stdout, temp_s1[i], order);
    }

    if (numOverflows > 0)
    {
        fprintf(stderr, "There was overflow - check the table\n");
    }
    fprintf(stderr, "[tools/sdk-tools/tabledesign.c] Exit tabledesign.c\n");
    return 0;
}
