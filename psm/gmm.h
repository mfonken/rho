//
//  gmm.h
//  GaussianMixtureModel
//
//  Created by Matthew Fonken on 2/9/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifdef __PSM__

#ifndef gmm_h
#define gmm_h

#ifdef __cplusplus
extern "C" {
#endif
    
#include "fsm.h"

/* Cluster functions */
void GaussianMixtureCluster_Initialize(                 gaussian_mixture_cluster_t *, observation_t *, vec2_t * );
void GaussianMixtureCluster_Update(                     gaussian_mixture_cluster_t *, observation_t *, vec2_t * );
void GaussianMixtureCluster_GetScore(                   gaussian_mixture_cluster_t *, vec2_t *);
void GaussianMixtureCluster_UpdateNormal(               gaussian_mixture_cluster_t * );
void GaussianMixtureCluster_UpdateInputProbability(     gaussian_mixture_cluster_t *, double );
void GaussianMixtureCluster_ContributeToOutput(         gaussian_mixture_cluster_t *, vec2_t *, vec2_t * );
void GaussianMixtureCluster_UpdateLimits(               gaussian_mixture_cluster_t * );
void GaussianMixtureCluster_Weigh(                      gaussian_mixture_cluster_t * );

/* Model functions */
void GaussianMixtureModel_Initialize(                   gaussian_mixture_model_t *, const char * );
double GaussianMixtureModel_GetScoreSumOfClusters(      gaussian_mixture_model_t *, vec2_t * );
double GaussianMixtureModel_GetOutputAndBestDistance(   gaussian_mixture_model_t *, double, vec2_t *, vec2_t * );
double GaussianMixtureModel_GetMaxError(                gaussian_mixture_model_t *, vec2_t *, vec2_t *, vec2_t * );
void GaussianMixtureModel_AddCluster(                   gaussian_mixture_model_t *, observation_t *, vec2_t * );
void GaussianMixtureModel_Update(                       gaussian_mixture_model_t *, observation_t *, vec2_t * );
void GaussianMixtureModel_AddValue(                     gaussian_mixture_model_t *, observation_t *, vec2_t * );
void GaussianMixtureModel_RemoveCluster(                gaussian_mixture_model_t *, uint16_t );

typedef struct
{
    void (*Initialize)(                 gaussian_mixture_cluster_t *, observation_t *, vec2_t * );
    void (*Update)(                     gaussian_mixture_cluster_t *, observation_t *, vec2_t * );
    void (*GetScore)(                   gaussian_mixture_cluster_t *, vec2_t *);
    void (*UpdateNormal)(               gaussian_mixture_cluster_t * );
    void (*UpdateInputProbability)(     gaussian_mixture_cluster_t *, double );
    void (*ContributeToOutput)(         gaussian_mixture_cluster_t *, vec2_t *, vec2_t * );
    void (*UpdateLimits)(               gaussian_mixture_cluster_t * );
    void (*Weigh)(                      gaussian_mixture_cluster_t * );
} gaussian_mixture_cluster_functions;

typedef struct
{
    void   (*Initialize)(               gaussian_mixture_model_t *, const char * );
    double (*GetScoreSumOfClusters)(    gaussian_mixture_model_t *, vec2_t * );
    double (*GetOutputAndBestDistance)( gaussian_mixture_model_t *, double, vec2_t *, vec2_t * );
    double (*GetMaxError)(              gaussian_mixture_model_t *, vec2_t *, vec2_t *, vec2_t * );
    void   (*AddCluster)(               gaussian_mixture_model_t *, observation_t *, vec2_t * );
    void   (*Update)(                   gaussian_mixture_model_t *, observation_t *, vec2_t * );
    void   (*AddValue)(                 gaussian_mixture_model_t *, observation_t *, vec2_t * );
    void   (*RemoveCluster)(            gaussian_mixture_model_t *, uint16_t );
} gaussian_mixture_model_functions;

typedef struct
{
    gaussian_mixture_cluster_functions Cluster;
    gaussian_mixture_model_functions Model;
} gaussian_mixture_functions;

static gaussian_mixture_functions GMMFunctions =
{
    .Cluster.Initialize             = GaussianMixtureCluster_Initialize,
    .Cluster.Update                 = GaussianMixtureCluster_Update,
    .Cluster.GetScore               = GaussianMixtureCluster_GetScore,
    .Cluster.UpdateNormal           = GaussianMixtureCluster_UpdateNormal,
    .Cluster.UpdateInputProbability = GaussianMixtureCluster_UpdateInputProbability,
    .Cluster.ContributeToOutput     = GaussianMixtureCluster_ContributeToOutput,
    .Cluster.UpdateLimits           = GaussianMixtureCluster_UpdateLimits,
    .Cluster.Weigh                  = GaussianMixtureCluster_Weigh,
    
    .Model.Initialize               = GaussianMixtureModel_Initialize,
    .Model.GetScoreSumOfClusters    = GaussianMixtureModel_GetScoreSumOfClusters,
    .Model.GetOutputAndBestDistance = GaussianMixtureModel_GetOutputAndBestDistance,
    .Model.GetMaxError              = GaussianMixtureModel_GetMaxError,
    .Model.AddCluster               = GaussianMixtureModel_AddCluster,
    .Model.Update                   = GaussianMixtureModel_Update,
    .Model.AddValue                 = GaussianMixtureModel_AddValue,
    .Model.RemoveCluster            = GaussianMixtureModel_RemoveCluster
};

#ifdef __cplusplus
}
#endif
    
#endif /* gmm_h */

#endif
