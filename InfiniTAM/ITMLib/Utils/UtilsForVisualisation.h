#ifndef UTILSFORVISUALISATION_H
#define UTILSFORVISUALISATION_H

#pragma once
#include "../ITMLib.h"
#include <sstream>
#include <vector>
#include <fstream>


//Kevin
#include "../../ORUtils/MemoryBlock.h"




namespace ITMLib
{
	/** \brief
	Provides functions for saving and loading scene states, saving scene to mesh etc.
	*/
  template <class TVoxel, class TIndex>
  class UtilsForVisualisation
  {
  public:
    UtilsForVisualisation(void){}
	
	/** \brief
	A point cloud of the current view is saved as a .ply file with its name "pointcloud_<frameNo>.ply"
	\param[in]  frameNo			Current frameNo
	\param[in]  trackingState	Pointer to an ITMTrackingState object
	\param[in]  imgSize_d	    Size of the depth image
	*/
	void SavePointCloudOfView(const int frameNo, const ITMTrackingState *trackingState, const Vector2i imgSize_d);  // get pointcloud representation of the scene from the current view
	
	
	/** \brief
	Generates a .txt file with filename "tsdf_<frameNo>.txt" on which marching cubes can be run to get the surface mesh
	\param[in]  frameNo		Current frameNo
	\param[in]  scene		Pointer to an ITMScene object
	*/
	void SaveTSDFforMC(const int frameNo, const ITMScene<TVoxel, TIndex> *scene);
	
	/** \brief
	Saves the current state of the scene, tracking state and rendering state to a file sceneState_<frameNo>_<HT/VA>.txt
	\param[in]  frameNo			Current frameNo
	\param[in]  scene			Pointer to an ITMScene object
	\param[in]  trackingState	Pointer to an ITMTrackingState object
	\param[in]  renderingState	Pointer to an ITMRenderingState object
	\param[in]  imgSize_d		Size of depth image
	*/
	void SaveCurrentState(const char *fileName, const ITMScene<TVoxel, TIndex> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, Vector2i imgSize_d);
	void SaveCurrentState_CompleteInfo(const char *fileName, const ITMScene<TVoxel, TIndex> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, Vector2i imgSize_d);
	
	/** \brief
	Loads the state of the scene, tracking state and rendering state from a saved file sceneState_<frameNo>_<HT/VA>.txt.
	\param[in]		frameNo							Current frameNo
	\param[out]  scene							Pointer to an ITMScene object
	\param[out]  trackingState					Pointer to an ITMTrackingState object
	\param[out]  renderingState					Pointer to an ITMRenderingState object
	\param[out]  hasStartedObjectReconstruction	Bool indicating if object reconstruction has started
	*/
	void LoadCurrentState(const char *fileName, const ITMScene<TVoxel, TIndex> *scene, const ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReruction = false);
	void LoadCurrentState_CompleteInfo(const char *fileName, const ITMScene<TVoxel, TIndex> *scene, const ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReruction = false);
  };

  /** \brief
  Dummy class that extends the class UtilsForVisualisation. This is specialized separately for
  ITMVoxelHashBlock and ITMPlainVoxelArray
  */
  template<class TVoxel,class TIndex>
  class UtilsForVis : public UtilsForVisualisation<TVoxel, TIndex>
  {
  };

  /** \brief
  Partially specilized class that extends the class UtilsForVisualisation for ITMVoxelHashBlock
  */
  template<class TVoxel>
  class UtilsForVis<TVoxel,ITMVoxelBlockHash> : public UtilsForVisualisation<TVoxel,ITMVoxelBlockHash>
  {
  public:
	void SaveTSDFforMC(const int frameNo, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene);
	void SaveCurrentState(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, const Vector2i imgSize_d);
	void SaveCurrentState_CompleteInfo(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, const Vector2i imgSize_d);
	void LoadCurrentState(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReconstruction);
	void LoadCurrentState_CompleteInfo(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReconstruction);
  };

  /** \brief
  Partially specilized class that extends the class UtilsForVisualisation for ITMPlainVoxelArray
  */
  template<class TVoxel>
  class UtilsForVis<TVoxel, ITMPlainVoxelArray> : public UtilsForVisualisation<TVoxel,ITMPlainVoxelArray>
  {
  public:
	// void SaveTSDFforMC(const int frameNo, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene);
	// void SaveCurrentState(const int frameNo, const ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, const Vector2i imgSize_d);
	// void LoadCurrentState(const int frameNo, ITMScene<TVoxel, ITMPlainVoxelArray> *scene, ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReconstruction);
	void SaveTSDFforMC(const int frameNo, ITMScene<TVoxel, ITMPlainVoxelArray> *scene);
	void SaveCurrentState(const char *fileName, ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, const Vector2i imgSize_d);
	void LoadCurrentState(const char *fileName, ITMScene<TVoxel, ITMPlainVoxelArray> *scene, ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReconstruction);
  };

}
#endif // UTILSFORVISUALISATION_H
