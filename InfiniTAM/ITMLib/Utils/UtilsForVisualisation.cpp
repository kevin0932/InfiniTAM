#include "UtilsForVisualisation.h"
using namespace ITMLib;

/** \brief
A point cloud of the current view is saved as a .ply file with its name "pointcloud_<frameNo>.ply"
\param[in]  frameNo			Current frameNo
\param[in]  trackingState	Pointer to an ITMTrackingState object
\param[in]  imgSize_d	    Size of the depth image
*/
template <class TVoxel, class TIndex>
void UtilsForVisualisation<TVoxel, TIndex>::SavePointCloudOfView(const int frameNo, const ITMTrackingState *trackingState, const Vector2i imgSize_d)
{
  // get a pointer to the point cloud locations data and color data
#ifndef COMPILE_WITHOUT_CUDA
  trackingState->pointCloud->locations->UpdateHostFromDevice();
  trackingState->pointCloud->colours->UpdateHostFromDevice();
#endif

  Vector4f *locations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
  Vector4f *colours = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);

  //int nDataSize = trackingState->pointCloud->noTotalPoints;
  int nDataSize = imgSize_d.width * imgSize_d.height;
  printf("no of depth pixels = %d\n", nDataSize);
  // write the data to a ply file
  std::stringstream ss_name;
  ss_name << "pointcloud_" << frameNo << ".ply";
  FILE *f = fopen(ss_name.str().c_str(), "w");

  fprintf(f, "ply\n");
  fprintf(f, "format ascii 1.0\n");
  fprintf(f, "element vertex %d\n", nDataSize);
  fprintf(f, "property float x\n");
  fprintf(f, "property float y\n");
  fprintf(f, "property float z\n");
  fprintf(f, "property float nx\n");
  fprintf(f, "property float ny\n");
  fprintf(f, "property float nz\n");
  fprintf(f, "end_header\n");

//  float maxDist = this->scene->sceneParams->viewFrustum_max;
//  float minDist = this->scene->sceneParams->viewFrustum_min;
//  float normFactor = maxDist - minDist;
  for (int i = 0; i < nDataSize; i++)
    {
	  if (locations[i].x == 0 && locations[i].y == 0 && locations[i].z == 0)
	  {
		  continue;
	  }

      fprintf(f, "%f %f %f ", locations[i].x, locations[i].y, locations[i].z);
      fprintf(f, "%f %f %f\n", colours[i].x, colours[i].y, colours[i].z);
    }
  fclose(f);
}


/** \brief
Generates a .txt file with filename "tsdf_<frameNo>.txt" on which marching cubes can be run to get the surface mesh
\param[in]  frameNo		Current frameNo
\param[in]  scene		Pointer to an ITMScene object
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMVoxelBlockHash>::SaveTSDFforMC(const int frameNo, const ITMScene<TVoxel, ITMVoxelBlockHash> *scene)
{
  std::cout << "Saving TSDF of scene. use marching cubes to create mesh" << std::endl;
  const ITMHashEntry *hashTable = scene->index.GetEntries();    // get pointer to the hash table (array of hash entries)
  const int numEntries = scene->index.noTotalEntries;      // total number of voxel blocks, both allocated and non-allocated (should be 9 * 2^17 for default settings in libDefines.h)
  const ITMVoxel *localVBA = scene->localVBA.GetVoxelBlocks();  // pointer to the local voxel blocks array

  int voxelBlocksX = 2048;             // number of voxel blocks along X
  int voxelBlocksY = 2048;             // number of voxel blocks along Y
  int voxelBlocksZ = 2048;             // number of voxel blocks along Z
  int voxelOffsetX = voxelBlocksX / 2;   // half of above value
  int voxelOffsetY = voxelBlocksY / 2;   // half of above value
  int voxelOffsetZ = voxelBlocksZ / 2;   // half of above value

  std::map<long long unsigned int, short> mapIndexAndTSDF;
  std::map<long long unsigned int, Vector3i > mapIndexAndPos;
  std::map<long long unsigned int, Vector3u > mapIndexAndColor;

  std::cout << "Total number of voxel blocks: " << numEntries << std::endl;
  int nValidEntries = 0;
  for (int i = 0; i < numEntries; i++)
    {
      // create a pointer to a ITMHashEntry object to store the retrieved entry from the hash table
      ITMHashEntry *hashEntry = new ITMHashEntry();
      // copy the corresponding entry from the hash table into the above object
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(hashEntry, &hashTable[i], sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
#else
      memcpy(hashEntry, &hashTable[i], sizeof(ITMHashEntry));
#endif
      // check validity of returned pointer
      if (!hashEntry->ptr)
        {
          continue;
        }
      if (hashEntry->ptr < 0) continue;
      nValidEntries++;

      // get global position of voxel block (one corner of the voxel block)
      Vector3i globalPosOfVoxelBlock;
      globalPosOfVoxelBlock.x = hashEntry->pos.x;
      globalPosOfVoxelBlock.y = hashEntry->pos.y;
      globalPosOfVoxelBlock.z = hashEntry->pos.z;
      globalPosOfVoxelBlock *= SDF_BLOCK_SIZE;

      // create a pointer to a ITMVoxel object to store the retrieved voxel block
      ITMVoxel *localVoxelBlock = (ITMVoxel*)malloc(sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
      // using the global position computed above, retrieve the voxel block from the localVBA and store it in the above object
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3), cudaMemcpyDeviceToHost));
#else
      memcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
#endif

      // loop over the voxel block to access each voxel within
      for (int z = 0; z < SDF_BLOCK_SIZE; z++)
        {
          for (int y = 0; y < SDF_BLOCK_SIZE; y++)
            {
              for (int x = 0; x < SDF_BLOCK_SIZE; x++)
                {
                  // get linear index into the voxel block
                  int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

                  // get index of the voxel in the global grid
                  Vector3i vIndex;
                  vIndex.x = globalPosOfVoxelBlock.x + x;
                  vIndex.y = globalPosOfVoxelBlock.y + y;
                  vIndex.z = globalPosOfVoxelBlock.z + z;

                  // push the voxel index and its associated tsdf value to a vector
                  if (localVoxelBlock[locId].sdf != 32767)
                    {// push only those voxels that were allocated
                      long long unsigned int linIndex = (vIndex.x + voxelOffsetX) + (vIndex.y + voxelOffsetY) * voxelBlocksX + (vIndex.z + voxelOffsetZ) * voxelBlocksY * voxelBlocksX;
                      // std::cout << vIndex.x << " " << vIndex.y << " " << vIndex.z << " " << std::endl;
                      // std::printf("%llu\n",linIndex);
                      mapIndexAndTSDF.insert(std::make_pair(linIndex, localVoxelBlock[locId].sdf));
                      mapIndexAndPos.insert(std::make_pair(linIndex, vIndex));
                      mapIndexAndColor.insert(std::make_pair(linIndex, localVoxelBlock[locId].clr));

                    }
                } //<-- end of loop over x
            } //<-- end of loop over y
        } //<-- end of loop over z
    } //<-- end of loop over numEntries

  std::cout << "Number of allocated voxel blocks:" << nValidEntries << std::endl;

  // work on the sdf volume
  std::stringstream ss_name;
  ss_name << "tsdf_" << frameNo << ".txt";
  FILE *f = fopen(ss_name.str().c_str(), "w");
  std::map<long long unsigned int, short>::iterator it;
  for (it = mapIndexAndTSDF.begin(); it != mapIndexAndTSDF.end(); it++)
    {
      //std::cout << it->first << std::endl;;

      // vector to store tsdf of 8 neighnours
      std::vector<short> ngbrTSDF;
      ngbrTSDF.push_back(it->second); // push tsdf of current voxel into the vector;

      // vector to store the color of the 8 neighbours
      std::vector<Vector3u> ngbrColor;
      ngbrColor.push_back(mapIndexAndColor[it->first]);

      // check if each of its 7 neighbours are in the map. If one of the 8 neighbours is not, then discard
      // here we check if the number of values in the map with a particular key are greater then 0
      if (mapIndexAndTSDF.count(it->first + 1) > 0){
          ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + 1));
          ngbrColor.push_back(mapIndexAndColor.at(it->first + 1));
        }
      else{ continue; }

      if (mapIndexAndTSDF.count(it->first + voxelBlocksX + 1) > 0){
		  ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelBlocksX + 1));
		  ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelBlocksX + 1));
        }
      else{ continue; }

	  if (mapIndexAndTSDF.count(it->first + voxelBlocksX) > 0){
		  ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelBlocksX));
		  ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelBlocksX));
        }
      else{ continue; }

	  if (mapIndexAndTSDF.count(it->first + voxelBlocksX * voxelBlocksY) > 0){
		  ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelBlocksX * voxelBlocksY));
		  ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelBlocksX * voxelBlocksY));
        }
      else{ continue; }

	  if (mapIndexAndTSDF.count(it->first + voxelBlocksX * voxelBlocksY + 1) > 0){
		  ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelBlocksX * voxelBlocksY + 1));
		  ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelBlocksX * voxelBlocksY + 1));
        }
      else{ continue; }

	  if (mapIndexAndTSDF.count(it->first + voxelBlocksX * voxelBlocksY + voxelBlocksX + 1) > 0){
		  ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelBlocksX * voxelBlocksY + voxelBlocksX + 1));
		  ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelBlocksX * voxelBlocksY + voxelBlocksX + 1));
        }
      else{ continue; }

	  if (mapIndexAndTSDF.count(it->first + voxelBlocksX * voxelBlocksY + voxelBlocksX) > 0){
		  ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelBlocksX * voxelBlocksY + voxelBlocksX));
		  ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelBlocksX * voxelBlocksY + voxelBlocksX));
        }
      else{ continue; }

      // if all 7 neighbours are present then write to file
      Vector3i pos = mapIndexAndPos[it->first];     // compute back indices
      fprintf(f, "%i %i %i", pos.x, pos.y, pos.z);    // write position to file
      for (int mm = 0; mm < ngbrTSDF.size(); mm++)   // write tsdf of all neighbours to file
        {
          fprintf(f, " %i", ngbrTSDF[mm]);
        }
      for (int mm = 0; mm < ngbrColor.size(); mm++)   // write tsdf of all neighbours to file
        {
          fprintf(f, " %u %u %u", ngbrColor[mm].x, ngbrColor[mm].y, ngbrColor[mm].z);
        }

      fprintf(f, "\n");
    }

  fclose(f);
  std::cout << "--- done" << std::endl;
  return;
}

/** \brief
Saves the current state of the scene, tracking state and rendering state to a file sceneState_<frameNo>_HT.txt
\param[in]  frameNo			Current frameNo
\param[in]  scene			Pointer to an ITMScene object
\param[in]  trackingState	Pointer to an ITMTrackingState object
\param[in]  renderingState	Pointer to an ITMRenderingState object
\param[in]  imgSize_d		Size of depth image
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMVoxelBlockHash>::SaveCurrentState(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, const Vector2i imgSize_d)
{
  std::ofstream ofs_sceneState;
  ofs_sceneState.open(fileName, std::ios::out);

  MemoryDeviceType useGPU;
#ifndef COMPILE_WITHOUT_CUDA
  useGPU = MEMORYDEVICE_CUDA;
#else
  useGPU = MEMORYDEVICE_CPU;
#endif

  ITMRenderState_VH *renderingState_vh = (ITMRenderState_VH*)renderingState;

  // get a few constants
  const int numEntries = scene->index.noTotalEntries;		// total number of voxel blocks, both allocated and non-allocated

  // print the current pose
  Vector3f translation;
  Vector3f rotation;
  trackingState->pose_d->GetParams(translation, rotation);
  ofs_sceneState << translation.x << " " << translation.y << " " << translation.z << " ";
  ofs_sceneState << rotation.x << " " << rotation.y << " " << rotation.z << std::endl;

  // print the pose_pointCloud
  trackingState->pose_pointCloud->GetParams(translation, rotation);
  ofs_sceneState << translation.x << " " << translation.y << " " << translation.z << " ";
  ofs_sceneState << rotation.x << " " << rotation.y << " " << rotation.z << std::endl;

  // print the age_pointCloud & requiresFullRendering in trackingState  #########################################################
  ofs_sceneState << trackingState->age_pointCloud << std::endl;
  ofs_sceneState << trackingState->requiresFullRendering << std::endl;
  ofs_sceneState << renderingState_vh->noFwdProjMissingPoints << std::endl;

  // print values from the tracking state and rendering state
  Vector4f *pointCloudLocations = trackingState->pointCloud->locations->GetData(useGPU);
  Vector4f *pointCloudNormals = trackingState->pointCloud->colours->GetData(useGPU);
  Vector4u *outRendering = renderingState_vh->raycastImage->GetData(useGPU);
  Vector2f *minMaxRange = renderingState_vh->renderingRangeImage->GetData(useGPU);
  Vector4f *raycastResultPtr = renderingState_vh->raycastResult->GetData(useGPU);
  Vector4f *forwardProjectionPtr = renderingState_vh->forwardProjection->GetData(useGPU);
  int *fwdProjMissingPointsPtr = renderingState_vh->fwdProjMissingPoints->GetData(useGPU);

  for (int index = 0; index < imgSize_d.width * imgSize_d.height; index++)
    {
      Vector4f location(0,0,0,-1);
      Vector4f normal;
      // Vector4f location, normal;
      Vector4u rendering;
      Vector2f minmax;
      Vector4f tmp_raycastResult;
      Vector4f tmp_forwardProjection;
      int tmp_fwdProjMissingPoints;
	  // TODO see if this can be moved outside the for loop. That will make it a single mem copy
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(location, pointCloudLocations[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(normal, pointCloudNormals[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(rendering, outRendering[index], sizeof(Vector4u), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(minmax, minMaxRange[index], sizeof(Vector2f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(tmp_raycastResult, raycastResultPtr[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(tmp_forwardProjection, forwardProjectionPtr[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  // ITMSafeCall(cudaMemcpy(tmp_fwdProjMissingPoints, fwdProjMissingPointsPtr[index], sizeof(int), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(&tmp_fwdProjMissingPoints, &fwdProjMissingPointsPtr[index], sizeof(int), cudaMemcpyDeviceToHost));
#else
      location = pointCloudLocations[index];
      normal = pointCloudNormals[index];
      rendering = outRendering[index];
      minmax = minMaxRange[index];
      tmp_raycastResult = raycastResultPtr[index];
      tmp_forwardProjection = forwardProjectionPtr[index];
      tmp_fwdProjMissingPoints = fwdProjMissingPointsPtr[index];      
#endif
      if (!(location.x == 0 && location.y == 0 && location.z == 0 && normal.r == 0 && normal.g == 0 && normal.b == 0))
        {
		  ofs_sceneState << index << " ";
          ofs_sceneState << (int)rendering.x << " ";	// this is a Vector4 with all values equal;
          ofs_sceneState << location.x << " ";
          ofs_sceneState << location.y << " ";
          ofs_sceneState << location.z << " ";
          ofs_sceneState << normal.r << " ";
          ofs_sceneState << normal.g << " ";
          ofs_sceneState << normal.b << " ";
          ofs_sceneState << minmax.x << " " << minmax.y << " ";

          ofs_sceneState << tmp_raycastResult.x << " " << tmp_raycastResult.y << " ";
          ofs_sceneState << tmp_raycastResult.z << " " << tmp_raycastResult.w << " ";
          ofs_sceneState << tmp_forwardProjection.x << " " << tmp_forwardProjection.y << " ";
          ofs_sceneState << tmp_forwardProjection.z << " " << tmp_forwardProjection.w << " ";
          ofs_sceneState << tmp_fwdProjMissingPoints << std::endl;
        }
    }

  // print some values regarding the hashTable and the localVBA
  ofs_sceneState << scene->localVBA.lastFreeBlockId << " ";
  ofs_sceneState << scene->index.GetLastFreeExcessListId() << std::endl;

  // print entriesVisibleType
  const uchar *entriesVisibleType = renderingState_vh->GetEntriesVisibleType();

  for (int i = 0; i < numEntries; i++)
    {// loop over all the entries
      uchar entryVisibleType;
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&entryVisibleType, &entriesVisibleType[i], sizeof(uchar), cudaMemcpyDeviceToHost));
#else
      entryVisibleType = entriesVisibleType[i];
#endif

      if (entryVisibleType == 0)
        {
          ofs_sceneState << i << " 0" << std::endl;
        }
      else if (entryVisibleType == 1)
        {
          ofs_sceneState << i << " 1" << std::endl;
        }
      else if (entryVisibleType == 2)
        {
          ofs_sceneState << i << " 2" << std::endl;
        }
      else if (entryVisibleType == 3)
        {
          ofs_sceneState << i << " 3" << std::endl;
        }
    }

  // print the visible entries IDs
  ofs_sceneState << renderingState_vh->noVisibleEntries << std::endl;
  const int *visibleEntryIDs = renderingState_vh->GetVisibleEntryIDs();

  for (int i = 0; i < renderingState_vh->noVisibleEntries; i++)
  {
      int visibleEntryID;
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&visibleEntryID, &visibleEntryIDs[i], sizeof(int), cudaMemcpyDeviceToHost));
#else
	  visibleEntryID = visibleEntryIDs[i];
#endif
	  ofs_sceneState << visibleEntryID << std::endl;
  }

  // print the local VBA (allocationList) and hash Table (excessAllocationList)
  const int *allocationList = scene->localVBA.GetAllocationList();	
  for(int i=0;i<scene->index.getNumAllocatedVoxelBlocks();i++)
  {
    int allocationListEntry;
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&allocationListEntry, &allocationList[i], sizeof(int), cudaMemcpyDeviceToHost));
#else
	  allocationListEntry = allocationList[i];
#endif

    if(i!=scene->index.getNumAllocatedVoxelBlocks()-1)
    {
      ofs_sceneState << allocationListEntry << " ";
    }
    else
    {
      ofs_sceneState << allocationListEntry << std::endl;
    }
  }

  const int *excessAllocationList = scene->index.GetExcessAllocationList();	
  for(int i=0;i<SDF_EXCESS_LIST_SIZE;i++)
  {
    int excessAllocationListEntry;
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&excessAllocationListEntry, &excessAllocationList[i], sizeof(int), cudaMemcpyDeviceToHost));
#else
	  excessAllocationListEntry = excessAllocationList[i];
#endif

    if(i!=SDF_EXCESS_LIST_SIZE-1)
    {
      ofs_sceneState << excessAllocationListEntry << " ";
    }
    else
    {
      ofs_sceneState << excessAllocationListEntry << std::endl;
    }
  }

  // print the local VBA and hash Table
  const ITMVoxel *localVBA = scene->localVBA.GetVoxelBlocks();	// pointer to the local voxel blocks array
  const ITMHashEntry *hashTable = scene->index.GetEntries();	// get pointer to the hashtable

  for (int i = 0; i < numEntries; i++)
    {// loop over all the entries
      // create a pointer to a ITMHashEntry object to store the retrieved entry from the hash table
      ITMHashEntry *hashEntry = new ITMHashEntry();
      // copy the corresponding entry from the hash table into the above object
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(hashEntry, &hashTable[i], sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
#else
      memcpy(hashEntry, &hashTable[i], sizeof(ITMHashEntry));
#endif
	  
      // // check validity of returned pointer
      if (!hashEntry->ptr || hashEntry->ptr < 0) continue;

      // get global position of the voxel block (one corner of the voxel block)
      Vector3i globalPosOfVoxelBlock;
      globalPosOfVoxelBlock.x = hashEntry->pos.x;
      globalPosOfVoxelBlock.y = hashEntry->pos.y;
      globalPosOfVoxelBlock.z = hashEntry->pos.z;
      //print the corner of the voxel block
	  ofs_sceneState << hashEntry->ptr << " ";
	  ofs_sceneState << hashEntry->offset << " ";
	  ofs_sceneState << globalPosOfVoxelBlock.x << " ";
      ofs_sceneState << globalPosOfVoxelBlock.y << " ";
      ofs_sceneState << globalPosOfVoxelBlock.z << std::endl;

      globalPosOfVoxelBlock *= SDF_BLOCK_SIZE;

      // create a pointer to a ITMVoxel object to store the retrieved voxel block
      ITMVoxel *localVoxelBlock = (ITMVoxel*)malloc(sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
      // using the global position computed above, retrieve the voxel block from the localVBA and store it in the above object
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3), cudaMemcpyDeviceToHost));
#else
      memcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
#endif

      // loop over the voxel block to access each voxel within
      for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
        {
          // get linear index into the voxel block
          int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

          // get position of the voxels
          // (these are in voxel units. to get values in metres multiply by voxelSize)
          Vector3f voxelPos;
          voxelPos.x = (float)(globalPosOfVoxelBlock.x + x);
          voxelPos.y = (float)(globalPosOfVoxelBlock.y + y);
          voxelPos.z = (float)(globalPosOfVoxelBlock.z + z);

          if (ITMVoxel::hasColorInformation)
            {
              // print the voxel position, its colour and SDF
              ofs_sceneState << (int)localVoxelBlock[locId].clr.r << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].clr.g << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].clr.b << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].w_color << " ";
              ofs_sceneState << localVoxelBlock[locId].sdf << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].w_depth << std::endl;
              // ofs_sceneState << (int)localVoxelBlock[locId].w_depth << " ";
              // ofs_sceneState << localVoxelBlock[locId].last_update_time << std::endl;
            }
        }

    }

  ofs_sceneState.close();
  std::cout << "scene save to " << fileName << std::endl;
}

/** \brief
Loads the state of the scene, tracking state and rendering state from a saved file sceneState_<frameNo>_HT.txt.
\param[in]		frameNo							Current frameNo
\param[out]  scene							Pointer to an ITMScene object
\param[out]  trackingState					Pointer to an ITMTrackingState object
\param[out]  renderingState					Pointer to an ITMRenderingState object
\param[out]  hasStartedObjectReconstruction	Bool indicating if object reconstruction has started (not used!)
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMVoxelBlockHash>::LoadCurrentState(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReconstruction)
{
  std::ifstream ifs_sceneState;
  ifs_sceneState.open(fileName, std::ios::in);

  ITMRenderState_VH* renderingState_vh = (ITMRenderState_VH*)renderingState;

  // load the current pose
  std::string line;
  std::getline(ifs_sceneState, line);
  std::stringstream ss(line);
  Vector3f translation;
  Vector3f rotation;
  ss >> translation.x >> translation.y >> translation.z;
  ss >> rotation.x >> rotation.y >> rotation.z;
  trackingState->pose_d->SetFrom(translation, rotation);


  // load the pose_pointCloud
  std::getline(ifs_sceneState, line);
  std::stringstream ss_pc(line);
  ss_pc >> translation.x >> translation.y >> translation.z;
  ss_pc >> rotation.x >> rotation.y >> rotation.z;
  trackingState->pose_pointCloud->SetFrom(translation, rotation);
  std::cout<<"pose_pointCloud loaded~~~"<<std::endl;

  // load the age_pointCloud & requiresFullRendering in trackingState  #########################################################
  std::getline(ifs_sceneState, line);
  std::stringstream ss_age_pointCloud(line);
  ss_age_pointCloud >> trackingState->age_pointCloud;
  std::getline(ifs_sceneState, line);
  std::stringstream ss_requiresFullRendering(line);
  ss_requiresFullRendering >> trackingState->requiresFullRendering;
  std::getline(ifs_sceneState, line);
  std::stringstream ss_noFwdProjMissingPoints(line);
  ss_noFwdProjMissingPoints >> renderingState_vh->noFwdProjMissingPoints;
  

  MemoryDeviceType useGPU;
#ifndef COMPILE_WITHOUT_CUDA
  useGPU = MEMORYDEVICE_CUDA;
#else
  useGPU = MEMORYDEVICE_CPU;
#endif
  // load the other stuff in the trackingState and rendering state (in CPU)
  Vector4f *pointCloudLocations = trackingState->pointCloud->locations->GetData(useGPU);
  Vector4f *pointCloudNormals = trackingState->pointCloud->colours->GetData(useGPU);
  Vector4u *outRendering = renderingState_vh->raycastImage->GetData(useGPU);
  Vector2f *minMaxRange = renderingState_vh->renderingRangeImage->GetData(useGPU);
  Vector4f *raycastResultPtr = renderingState_vh->raycastResult->GetData(useGPU);
  Vector4f *forwardProjectionPtr = renderingState_vh->forwardProjection->GetData(useGPU);
  int *fwdProjMissingPointsPtr = renderingState_vh->fwdProjMissingPoints->GetData(useGPU);

  while (std::getline(ifs_sceneState, line))
    {
      Vector4f location, normal;
      Vector4u rendering;
      Vector2f minmax;
      Vector4f tmp_raycastResult;
      Vector4f tmp_forwardProjection;
      int tmp_fwdProjMissingPoints;      

      int index, render;
      std::stringstream ss1(line);

      if (!(ss1 >> index >> render >> location.x >> location.y >> location.z >> normal.r >> normal.g >> normal.b >> minmax.x >> minmax.y >> tmp_raycastResult.x >> tmp_raycastResult.y >> tmp_raycastResult.z >> tmp_raycastResult.w >> tmp_forwardProjection.x >> tmp_forwardProjection.y >> tmp_forwardProjection.z >> tmp_forwardProjection.w >> tmp_fwdProjMissingPoints))
        {
          // load the values of lastFreeBlockId and lastFreeExcessListId
          scene->localVBA.lastFreeBlockId = index;
		  scene->index.SetLastFreeExcessListId(render);
          break;
        }

      location.w = 0; normal.w = 0;

      rendering.x = (uchar)render;
      rendering.y = (uchar)render;
      rendering.z = (uchar)render;
      rendering.w = (uchar)render;
  
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&pointCloudLocations[index], &location, sizeof(Vector4f), cudaMemcpyHostToDevice));
      // ITMSafeCall(cudaMemcpy(pointCloudLocations[index], location, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&pointCloudNormals[index], &normal, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&outRendering[index], &rendering, sizeof(Vector4u), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&minMaxRange[index], &minmax, sizeof(Vector2f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&raycastResultPtr[index], &tmp_raycastResult, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&forwardProjectionPtr[index], &tmp_forwardProjection, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&fwdProjMissingPointsPtr[index], &tmp_fwdProjMissingPoints, sizeof(int), cudaMemcpyHostToDevice));
#else
      // // memcpy(&pointCloudLocations[index], &location, sizeof(Vector4f));
      // memcpy(pointCloudLocations[index], location, sizeof(Vector4f));
      // memcpy(pointCloudNormals[index], normal, sizeof(Vector4f));
      // memcpy(outRendering[index], rendering, sizeof(Vector4u));
      // memcpy(minMaxRange[index], minmax, sizeof(Vector2f));
      // memcpy(raycastResultPtr[index], tmp_raycastResult, sizeof(Vector4f));
      // memcpy(forwardProjectionPtr[index], tmp_forwardProjection, sizeof(Vector4f));
      // memcpy(&fwdProjMissingPointsPtr[index], &tmp_fwdProjMissingPoints, sizeof(int));  
      pointCloudLocations[index] = location;
      pointCloudNormals[index] = normal;
      outRendering[index] = rendering;
      minMaxRange[index] = minmax;
      raycastResultPtr[index] = tmp_raycastResult;
      forwardProjectionPtr[index] = tmp_forwardProjection;
      fwdProjMissingPointsPtr[index] = tmp_fwdProjMissingPoints;
#endif  
    }

  #ifndef COMPILE_WITHOUT_CUDA
    trackingState->pointCloud->locations->UpdateHostFromDevice();
    trackingState->pointCloud->colours->UpdateHostFromDevice();
    renderingState_vh->raycastImage->UpdateHostFromDevice();
    renderingState_vh->renderingRangeImage->UpdateHostFromDevice();

    renderingState_vh->raycastResult->UpdateHostFromDevice();
    renderingState_vh->forwardProjection->UpdateHostFromDevice();
    renderingState_vh->fwdProjMissingPoints->UpdateHostFromDevice();  
  #endif


  // load the values of entries visible type
  uchar *entriesVisibleType = renderingState_vh->GetEntriesVisibleType();
  while (std::getline(ifs_sceneState, line))
    {
      int index;
      int val;
      std::stringstream ss2(line);

      if (!(ss2 >> index >> val))
        {// load the number of live indexIds
		  renderingState_vh->noVisibleEntries = index;
          break;
        }
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&entriesVisibleType[index], &val, sizeof(uchar), cudaMemcpyHostToDevice));
#else
      entriesVisibleType[index] = val;
#endif
    }

std::cout << "live entries: " << renderingState_vh->noVisibleEntries << std::endl;

  // load the values of live entry ids
int *visibleEntryIDs = renderingState_vh->GetVisibleEntryIDs();
int visibleEntryID;
  for (int i = 0; i < renderingState_vh->noVisibleEntries; i++)
    {
      std::getline(ifs_sceneState, line);
	  visibleEntryID = atoi(line.c_str());

#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&visibleEntryIDs[i], &visibleEntryID, sizeof(int), cudaMemcpyHostToDevice));
#else
	  visibleEntryIDs[i] = visibleEntryID;
#endif

    }
std::cout << "Contents of live entries: " << std::endl;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  int *allocationList = scene->localVBA.GetAllocationList();
  std::getline(ifs_sceneState, line);
  std::stringstream ss9(line);
  int *tmpAllocationList = (int*) malloc(sizeof(int));
  for(int i=0;i<scene->index.getNumAllocatedVoxelBlocks();i++)
  {
    ss9 >> *tmpAllocationList;
#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&(allocationList[i]), tmpAllocationList, sizeof(int), cudaMemcpyHostToDevice));
#else
    memcpy(&(allocationList[i]), tmpAllocationList, sizeof(int));
#endif
  }

  int *excessAllocationList = scene->index.GetExcessAllocationList();
  std::getline(ifs_sceneState, line);
  std::stringstream ss10(line);
  int *tmpExcessAllocationList = (int*) malloc(sizeof(int));
  for(int i=0;i<SDF_EXCESS_LIST_SIZE;i++)
  {
    ss10 >> *tmpExcessAllocationList;
#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&(excessAllocationList[i]), tmpExcessAllocationList, sizeof(int), cudaMemcpyHostToDevice));
#else
    memcpy(&(excessAllocationList[i]), tmpExcessAllocationList, sizeof(int));
#endif
  }
// // Put outside loop -> buffer overflow error! 
//         // std::cout<<"BEFORE! fill allocationList "<<std::endl;
// #ifndef COMPILE_WITHOUT_CUDA
//       ITMSafeCall(cudaMemcpy(&(allocationList), tmpAllocationList, sizeof(int)* (scene->index.getNumAllocatedVoxelBlocks()), cudaMemcpyHostToDevice));
// #else
//       memcpy(&(allocationList), tmpAllocationList, sizeof(int)* (scene->index.getNumAllocatedVoxelBlocks()));
// #endif
//       // std::cout<<"AFTER! fill allocationList "<<std::endl;
//       // std::cout<<"BEFORE! fill excessAllocationList"<<std::endl;
// #ifndef COMPILE_WITHOUT_CUDA
//       ITMSafeCall(cudaMemcpy(&(excessAllocationList), tmpExcessAllocationList, sizeof(int)* (SDF_EXCESS_LIST_SIZE), cudaMemcpyHostToDevice));
// #else
//       memcpy(&(excessAllocationList), tmpExcessAllocationList, sizeof(int)* (SDF_EXCESS_LIST_SIZE));
// #endif
//       // std::cout<<"AFTER! fill excessAllocationList"<<std::endl;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~// 

  // load the local VBA and the hash table
  ITMVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
  ITMHashEntry *hashTable = scene->index.GetEntries();
  const int numEntries = scene->index.noTotalEntries;	// total number of voxel blocks

  for (int i = 0; i < numEntries; i++)	// this loop actually goes only over number of valid entries
    {
      // create a new hash table entry
      ITMHashEntry *hashEntry = new ITMHashEntry();
      std::getline(ifs_sceneState, line);
      std::stringstream ss3(line);
      if (!(ss3 >> hashEntry->ptr >> hashEntry->offset >> hashEntry->pos.x >> hashEntry->pos.y >> hashEntry->pos.z))
        {
          std::cout<<"BREAK! load localVBA and HashTable Loop --- Entry "<<i<<"/"<<numEntries<<std::endl;
          break;
        }

      // localVoxelBlock that contains a block of 8x8x8 voxels. this will be pushed into the localVBA
      ITMVoxel *localVoxelBlock = (ITMVoxel*)malloc(sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));

      for (int locind = 0; locind < SDF_BLOCK_SIZE3; locind++)
        {
          int r, g, b, w_color, w_depth;
          short sdf;
          // float last_update_time;
          std::getline(ifs_sceneState, line);
          std::stringstream ss4(line);

          ss4 >> r >> g >> b >> w_color >> sdf >> w_depth;// >> last_update_time;
          // fill the local 8x8x8 voxel block
          localVoxelBlock[locind].clr.r = (uchar)r;
          localVoxelBlock[locind].clr.g = (uchar)g;
          localVoxelBlock[locind].clr.b = (uchar)b;
          localVoxelBlock[locind].w_color = (uchar)w_color;
          localVoxelBlock[locind].sdf = sdf;
          localVoxelBlock[locind].w_depth = (uchar)w_depth;
          // localVoxelBlock[locind].last_update_time = last_update_time;
        }

      // fill the local VBA with this block
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), localVoxelBlock, sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3), cudaMemcpyHostToDevice));
#else
      memcpy(&(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), localVoxelBlock, sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
#endif

      // fill hash table with the above hash entry
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
#else
      memcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry));
#endif
    }

  ifs_sceneState.close();
  std::cout << "loading from " << fileName << " is done :)" << std::endl;
}

/** \brief
Generates a .txt file with filename "tsdf_<frameNo>.txt" on which marching cubes can be run to get the surface mesh
\param[in]  frameNo		Current frameNo
\param[in]  scene		Pointer to an ITMScene object
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMPlainVoxelArray>::SaveTSDFforMC(const int frameNo, ITMScene<TVoxel, ITMPlainVoxelArray> *scene)
{
	std::cout << "Saving TSDF of scene. use marching cubes to create mesh" << std::endl;

	const ITMVoxel *voxelArray = scene->localVBA.GetVoxelBlocks();	// pointer to voxel array
	const Vector3i voxelArraySize = scene->index.getVolumeSize();	// size of the voxel array

	std::map<long long unsigned int, short> mapIndexAndTSDF;
	std::map<long long unsigned int, Vector3i > mapIndexAndPos;
	std::map<long long unsigned int, Vector3u > mapIndexAndColor;

	// loop over the voxel block to access each voxel within
	for (int z = 0; z < voxelArraySize.z; z++)
	{
		for (int y = 0; y < voxelArraySize.y; y++)
		{
			for (int x = 0; x < voxelArraySize.x; x++)
			{
				// get linear index into the voxel array
				int locId = x + y * voxelArraySize.x + z * voxelArraySize.y * voxelArraySize.x;

				ITMVoxel *voxel_host = new ITMVoxel();
#ifndef COMPILE_WITHOUT_CUDA
				ITMSafeCall(cudaMemcpy(voxel_host, &voxelArray[locId], sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
#else
				memcpy(voxel_host, &voxelArray[locId], sizeof(ITMVoxel));
#endif
				if (voxel_host->sdf != 32767)
				{// push only those voxels that were allocated
					mapIndexAndTSDF.insert(std::make_pair(locId, voxel_host->sdf));
					mapIndexAndPos.insert(std::make_pair(locId, Vector3i(x,y,z)));
					mapIndexAndColor.insert(std::make_pair(locId, voxel_host->clr));
				}
			} //<-- end of loop over x
		} //<-- end of loop over y
	} //<-- end of loop over z

	// work on the sdf volume
	std::stringstream ss_name;
	ss_name << "tsdf_" << frameNo << ".txt";
	FILE *f = fopen(ss_name.str().c_str(), "w");
	std::map<long long unsigned int, short>::iterator it;
	for (it = mapIndexAndTSDF.begin(); it != mapIndexAndTSDF.end(); it++)
	{
		//std::cout << it->first << std::endl;;

		// vector to store tsdf of 8 neighnours
		std::vector<short> ngbrTSDF;
		ngbrTSDF.push_back(it->second); // push tsdf of current voxel into the vector;

		// vector to store the color of the 8 neighbours
		std::vector<Vector3u> ngbrColor;
		ngbrColor.push_back(mapIndexAndColor[it->first]);

		// check if each of its 7 neighbours are in the map. If one of the 8 neighbours is not, then discard
		// here we check if the number of values in the map with a particular key are greater then 0
		if (mapIndexAndTSDF.count(it->first + 1) > 0){
			ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + 1));
			ngbrColor.push_back(mapIndexAndColor.at(it->first + 1));
		}
		else{ continue; }

		if (mapIndexAndTSDF.count(it->first + voxelArraySize.x + 1) > 0){
			ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelArraySize.x + 1));
			ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelArraySize.x + 1));
		}
		else{ continue; }

		if (mapIndexAndTSDF.count(it->first + voxelArraySize.x) > 0){
			ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelArraySize.x));
			ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelArraySize.x));
		}
		else{ continue; }

		if (mapIndexAndTSDF.count(it->first + voxelArraySize.x * voxelArraySize.y) > 0){
			ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelArraySize.x * voxelArraySize.y));
			ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelArraySize.x * voxelArraySize.y));
		}
		else{ continue; }

		if (mapIndexAndTSDF.count(it->first + voxelArraySize.x * voxelArraySize.y + 1) > 0){
			ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelArraySize.x * voxelArraySize.y + 1));
			ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelArraySize.x * voxelArraySize.y + 1));
		}
		else{ continue; }

		if (mapIndexAndTSDF.count(it->first + voxelArraySize.x * voxelArraySize.y + voxelArraySize.x + 1) > 0){
			ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelArraySize.x * voxelArraySize.y + voxelArraySize.x + 1));
			ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelArraySize.x * voxelArraySize.y + voxelArraySize.x + 1));
		}
		else{ continue; }

		if (mapIndexAndTSDF.count(it->first + voxelArraySize.x * voxelArraySize.y + voxelArraySize.x) > 0){
			ngbrTSDF.push_back(mapIndexAndTSDF.at(it->first + voxelArraySize.x * voxelArraySize.y + voxelArraySize.x));
			ngbrColor.push_back(mapIndexAndColor.at(it->first + voxelArraySize.x * voxelArraySize.y + voxelArraySize.x));
		}
		else{ continue; }

		// if all 7 neighbours are present then write to file
		Vector3i pos = mapIndexAndPos[it->first];     // compute back indices
		fprintf(f, "%i %i %i", pos.x, pos.y, pos.z);    // write position to file
		for (int mm = 0; mm < ngbrTSDF.size(); mm++)   // write tsdf of all neighbours to file
		{
			fprintf(f, " %i", ngbrTSDF[mm]);
		}
		for (int mm = 0; mm < ngbrColor.size(); mm++)   // write tsdf of all neighbours to file
		{
			fprintf(f, " %u %u %u", ngbrColor[mm].x, ngbrColor[mm].y, ngbrColor[mm].z);
		}

		fprintf(f, "\n");
	}

	fclose(f);
	std::cout << "--- done" << std::endl;
	return;
}

/** \brief
Saves the current state of the scene, tracking state and rendering state to a file sceneState_<frameNo>_VA.txt
\param[in]  frameNo			Current frameNo
\param[in]  scene			Pointer to an ITMScene object
\param[in]  trackingState	Pointer to an ITMTrackingState object
\param[in]  renderingState	Pointer to an ITMRenderingState object
\param[in]  imgSize_d		Size of depth image
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMPlainVoxelArray>::SaveCurrentState(const char *fileName, ITMScene<TVoxel, ITMPlainVoxelArray> *scene, const ITMTrackingState *trackingState, const ITMRenderState* renderingState, const Vector2i imgSize_d)
{
	std::ofstream ofs_sceneState;
  ofs_sceneState.open(fileName, std::ios::out);

	MemoryDeviceType useGPU;
#ifndef COMPILE_WITHOUT_CUDA
	useGPU = MEMORYDEVICE_CUDA;
#else
	useGPU = MEMORYDEVICE_CPU;
#endif

	// print the current pose
	Vector3f translation;
	Vector3f rotation;
	trackingState->pose_d->GetParams(translation, rotation);
	ofs_sceneState << translation.x << " " << translation.y << " " << translation.z << " ";
	ofs_sceneState << rotation.x << " " << rotation.y << " " << rotation.z << std::endl;

	// print values from the tracking state
	Vector4f *pointCloudLocations = trackingState->pointCloud->locations->GetData(useGPU);
	Vector4f *pointCloudNormals = trackingState->pointCloud->colours->GetData(useGPU);
	Vector4u *outRendering = renderingState->raycastImage->GetData(useGPU);
	Vector2f *minMaxRange = renderingState->renderingRangeImage->GetData(useGPU);

	for (int index = 0; index < imgSize_d.width * imgSize_d.height; index++)
	{
		Vector4f location, normal;
		Vector4u rendering;
		Vector2f minmax;
		// TODO see if this can be moved outside the for loop. That will make it a single mem copy
#ifndef COMPILE_WITHOUT_CUDA
		ITMSafeCall(cudaMemcpy(location, pointCloudLocations[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(normal, pointCloudNormals[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(rendering, outRendering[index], sizeof(Vector4u), cudaMemcpyDeviceToHost));
		ITMSafeCall(cudaMemcpy(minmax, minMaxRange[index], sizeof(Vector2f), cudaMemcpyDeviceToHost));
#else
		location = pointCloudLocations[index];
		normal = pointCloudNormals[index];
		rendering = outRendering[index];
		minmax = minMaxRange[index];
#endif
		if (!(location.x == 0 && location.y == 0 && location.z == 0 && normal.r == 0 && normal.g == 0 && normal.b == 0))
		{
			ofs_sceneState << index << " ";
			ofs_sceneState << (int)rendering.x << " ";	// this is a Vector4 with all values equal;
			ofs_sceneState << location.x << " ";
			ofs_sceneState << location.y << " ";
			ofs_sceneState << location.z << " ";
			ofs_sceneState << normal.r << " ";
			ofs_sceneState << normal.g << " ";
			ofs_sceneState << normal.b << " ";
			ofs_sceneState << minmax.x << " " << minmax.y << std::endl;
		}
	}

	
	// print values from the scene
	Vector3i voxelArraySize = scene->index.getVolumeSize();
	Vector3i voxelOffset = scene->index.getOffset();
	ofs_sceneState << voxelArraySize.x << " " << voxelArraySize.y << " " << voxelArraySize.z << " ";
	ofs_sceneState << voxelOffset.x << " " << voxelOffset.y << " " << voxelOffset.z << std::endl;

	// loop over the voxel block to access each voxel within
	const ITMVoxel *voxelBlock = scene->localVBA.GetVoxelBlocks();
	for (int z = 0; z < voxelArraySize.z; z++)
	{
		for (int y = 0; y < voxelArraySize.y; y++)
		{
			for (int x = 0; x < voxelArraySize.x; x++)
			{
				// get linear index into the voxel block
				int locId = x + y * voxelArraySize.x + z * voxelArraySize.y * voxelArraySize.x;

				// copy the voxel content from GPU to CPU if necessary
				ITMVoxel *voxelBlock_host = new ITMVoxel();
#ifndef COMPILE_WITHOUT_CUDA
				ITMSafeCall(cudaMemcpy(voxelBlock_host, &voxelBlock[locId], sizeof(ITMVoxel), cudaMemcpyDeviceToHost));
#else
				memcpy(voxelBlock_host, &voxelBlock[locId], sizeof(ITMVoxel));
#endif
				if (voxelBlock_host->sdf != 32767)
				{
					ofs_sceneState << locId << " ";
					ofs_sceneState << (int)voxelBlock_host->clr.r << " ";
					ofs_sceneState << (int)voxelBlock_host->clr.g << " ";
					ofs_sceneState << (int)voxelBlock_host->clr.b << " ";
					ofs_sceneState << voxelBlock_host->sdf << std::endl;
				}

			}
		}
	}

	ofs_sceneState.close();
	std::cout << "scene save to " << fileName << std::endl;
}

/** \brief
Loads the state of the scene, tracking state and rendering state from a saved file sceneState_<frameNo>_VA.txt.
\param[in]		frameNo							Current frameNo
\param[out]  scene							Pointer to an ITMScene object
\param[out]  trackingState					Pointer to an ITMTrackingState object
\param[out]  renderingState					Pointer to an ITMRenderingState object
\param[out]  hasStartedObjectReconstruction	Bool indicating if object reconstruction has started
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMPlainVoxelArray>::LoadCurrentState(const char *fileName, ITMScene<TVoxel, ITMPlainVoxelArray> *scene, ITMTrackingState *trackingState, ITMRenderState* renderingState, bool *hasStartedObjectReconstruction)
{
	std::cout << "debug 1 done" << std::endl;
	// open the desired file
  std::ifstream ifs_sceneState;
  ifs_sceneState.open(fileName, std::ios::in);
	std::cout << "debug 2 done" << std::endl;

	// load the current pose
	std::string line;
	std::getline(ifs_sceneState, line);
	std::stringstream ss(line);
	Vector3f translation;
	Vector3f rotation;
	ss >> translation.x >> translation.y >> translation.z;
	ss >> rotation.x >> rotation.y >> rotation.z;
	trackingState->pose_d->SetFrom(translation, rotation);
	std::cout << "debug 3 done" << std::endl;

	// // load the other stuff in the trackingState and rendering state (in CPU)
	// Vector4f *pointCloudLocations = trackingState->pointCloud->locations->GetData(MEMORYDEVICE_CPU);
	// Vector4f *pointCloudNormals = trackingState->pointCloud->colours->GetData(MEMORYDEVICE_CPU);
	// Vector4u *outRendering = renderingState->raycastImage->GetData(MEMORYDEVICE_CPU);
	// Vector2f *minMaxRange = renderingState->renderingRangeImage->GetData(MEMORYDEVICE_CPU);
   MemoryDeviceType useGPU;
#ifndef COMPILE_WITHOUT_CUDA
  useGPU = MEMORYDEVICE_CUDA;
#else
  useGPU = MEMORYDEVICE_CPU;
#endif
  // load the other stuff in the trackingState and rendering state (in CPU)
  Vector4f *pointCloudLocations = trackingState->pointCloud->locations->GetData(useGPU);
  Vector4f *pointCloudNormals = trackingState->pointCloud->colours->GetData(useGPU);
  Vector4u *outRendering = renderingState->raycastImage->GetData(useGPU);
  Vector2f *minMaxRange = renderingState->renderingRangeImage->GetData(useGPU);
	
	ITMPlainVoxelArray::IndexData *voxelIndex_host = scene->index.getIndexData(MEMORYDEVICE_CPU);
	std::cout << "debug 4 done" << std::endl;
	while (std::getline(ifs_sceneState, line))
	{
		Vector4f location, normal;
		Vector4u rendering;
		Vector2f minmax;

		int index, render;
		std::stringstream ss1(line);

		if (!(ss1 >> index >> render >> location.x >> location.y >> location.z >> normal.r >> normal.g >> normal.b >> minmax.x >> minmax.y))
		{
			// print values from the scene
			voxelIndex_host->size.x = index;
			voxelIndex_host->size.y = render;
			voxelIndex_host->size.z = location.x;

			voxelIndex_host->offset.x = location.y;
			voxelIndex_host->offset.y = location.z;
			voxelIndex_host->offset.z = (int)normal.r;
			
#ifndef COMPILE_WITHOUT_CUDA
			// ITMPlainVoxelArray::IndexData *voxelIndex = scene->index.getIndexData(MEMORYDEVICE_CUDA);
			// ITMSafeCall(cudaMemcpy(voxelIndex, voxelIndex_host, sizeof(ITMPlainVoxelArray::IndexData), cudaMemcpyHostToDevice));
      // scene->index.getIndexData(MEMORYDEVICE_CUDA)->UpdateDeviceFromHost();
      scene->index.UpdateDeviceDataWithHostData();
#endif
			break;
		}

		location.w = 0; normal.w = 0;

		rendering.x = (uchar)render;
		rendering.y = (uchar)render;
		rendering.z = (uchar)render;
		rendering.w = (uchar)render;
#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&pointCloudLocations[index], &location, sizeof(Vector4f), cudaMemcpyHostToDevice));
    // ITMSafeCall(cudaMemcpy(pointCloudLocations[index], location, sizeof(Vector4f), cudaMemcpyHostToDevice));
    ITMSafeCall(cudaMemcpy(&pointCloudNormals[index], &normal, sizeof(Vector4f), cudaMemcpyHostToDevice));
    ITMSafeCall(cudaMemcpy(&outRendering[index], &rendering, sizeof(Vector4u), cudaMemcpyHostToDevice));
    ITMSafeCall(cudaMemcpy(&minMaxRange[index], &minmax, sizeof(Vector2f), cudaMemcpyHostToDevice));
#else
		memcpy(pointCloudLocations[index], location, sizeof(Vector4f));
		memcpy(pointCloudNormals[index], normal, sizeof(Vector4f));
		memcpy(outRendering[index], rendering, sizeof(Vector4u));
		memcpy(minMaxRange[index], minmax, sizeof(Vector2f));
#endif
	}
	std::cout << "debug 5 done" << std::endl;

// #ifndef COMPILE_WITHOUT_CUDA
// 	trackingState->pointCloud->locations->UpdateDeviceFromHost();
// 	trackingState->pointCloud->colours->UpdateDeviceFromHost();
// 	renderingState->raycastImage->UpdateDeviceFromHost();
// 	renderingState->renderingRangeImage->UpdateDeviceFromHost();
// #endif

	ITMVoxel* voxelBlock = scene->localVBA.GetVoxelBlocks();
	std::cout << "debug 6 done" << std::endl;

	// load the Voxel Array
	while (std::getline(ifs_sceneState, line))
	{
		int locId, r, g, b;
		short sdf;
		std::stringstream ss2(line);

		if (!(ss2 >> locId >> r >> g >> b >> sdf))
		{
			break;
		}
		// fill the voxel array at the appropriate index
		ITMVoxel *voxelBlock_host = new ITMVoxel();
		voxelBlock_host->sdf = sdf;
		voxelBlock_host->clr.r = r;
		voxelBlock_host->clr.g = g;
		voxelBlock_host->clr.b = b;

#ifndef COMPILE_WITHOUT_CUDA
		ITMSafeCall(cudaMemcpy(&voxelBlock[locId], voxelBlock_host, sizeof(ITMVoxel), cudaMemcpyHostToDevice));
#else
		memcpy(&voxelBlock[locId], voxelBlock_host, sizeof(ITMVoxel));
#endif
	}
	std::cout << "debug 7 done" << std::endl;

	ifs_sceneState.close();
	std::cout << "loading done" << std::endl;
}




template class UtilsForVisualisation<ITMVoxel, ITMVoxelIndex>;
template class UtilsForVis<ITMVoxel, ITMVoxelIndex>;



//#########################################################################################################################################//
//#########################################################################################################################################//
//#########################################################################################################################################//
//#########################################################################################################################################//
/** \brief
Saves the current state of the scene, tracking state and rendering state to a file sceneState_<frameNo>_HT.txt
\param[in]  frameNo			Current frameNo
\param[in]  scene			Pointer to an ITMScene object
\param[in]  trackingState	Pointer to an ITMTrackingState object
\param[in]  renderingState	Pointer to an ITMRenderingState object
\param[in]  imgSize_d		Size of depth image
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMVoxelBlockHash>::SaveCurrentState_CompleteInfo(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, const ITMTrackingState *trackingState, const ITMRenderState *renderingState, const Vector2i imgSize_d)
{
  std::ofstream ofs_sceneState;
  ofs_sceneState.open(fileName, std::ios::out);

  MemoryDeviceType useGPU;
#ifndef COMPILE_WITHOUT_CUDA
  useGPU = MEMORYDEVICE_CUDA;
#else
  useGPU = MEMORYDEVICE_CPU;
#endif

  ITMRenderState_VH *renderingState_vh = (ITMRenderState_VH*)renderingState;

  // get a few constants
  const int numEntries = scene->index.noTotalEntries;		// total number of voxel blocks, both allocated and non-allocated

  // print the current pose
  Vector3f translation;
  Vector3f rotation;
  trackingState->pose_d->GetParams(translation, rotation);
  ofs_sceneState << translation.x << " " << translation.y << " " << translation.z << " ";
  ofs_sceneState << rotation.x << " " << rotation.y << " " << rotation.z << std::endl;

  // print the pose_pointCloud
  trackingState->pose_pointCloud->GetParams(translation, rotation);
  ofs_sceneState << translation.x << " " << translation.y << " " << translation.z << " ";
  ofs_sceneState << rotation.x << " " << rotation.y << " " << rotation.z << std::endl;

  // print the age_pointCloud & requiresFullRendering in trackingState  #########################################################
  ofs_sceneState << trackingState->age_pointCloud << std::endl;
  ofs_sceneState << trackingState->requiresFullRendering << std::endl;
  ofs_sceneState << renderingState_vh->noFwdProjMissingPoints << std::endl;

  // print values from the tracking state and rendering state
  Vector4f *pointCloudLocations = trackingState->pointCloud->locations->GetData(useGPU);
  Vector4f *pointCloudNormals = trackingState->pointCloud->colours->GetData(useGPU);
  Vector4u *outRendering = renderingState_vh->raycastImage->GetData(useGPU);
  Vector2f *minMaxRange = renderingState_vh->renderingRangeImage->GetData(useGPU);
  Vector4f *raycastResultPtr = renderingState_vh->raycastResult->GetData(useGPU);
  Vector4f *forwardProjectionPtr = renderingState_vh->forwardProjection->GetData(useGPU);
  int *fwdProjMissingPointsPtr = renderingState_vh->fwdProjMissingPoints->GetData(useGPU);

  for (int index = 0; index < imgSize_d.width * imgSize_d.height; index++)
    {
      Vector4f location(0,0,0,-1);
      Vector4f normal;
      // Vector4f location, normal;
      Vector4u rendering;
      Vector2f minmax;
      Vector4f tmp_raycastResult;
      Vector4f tmp_forwardProjection;
      int tmp_fwdProjMissingPoints;
	  // TODO see if this can be moved outside the for loop. That will make it a single mem copy
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(location, pointCloudLocations[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(normal, pointCloudNormals[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(rendering, outRendering[index], sizeof(Vector4u), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(minmax, minMaxRange[index], sizeof(Vector2f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(tmp_raycastResult, raycastResultPtr[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(tmp_forwardProjection, forwardProjectionPtr[index], sizeof(Vector4f), cudaMemcpyDeviceToHost));
	  // ITMSafeCall(cudaMemcpy(tmp_fwdProjMissingPoints, fwdProjMissingPointsPtr[index], sizeof(int), cudaMemcpyDeviceToHost));
	  ITMSafeCall(cudaMemcpy(&tmp_fwdProjMissingPoints, &fwdProjMissingPointsPtr[index], sizeof(int), cudaMemcpyDeviceToHost));
#else
      location = pointCloudLocations[index];
      normal = pointCloudNormals[index];
      rendering = outRendering[index];
      minmax = minMaxRange[index];
      tmp_raycastResult = raycastResultPtr[index];
      tmp_forwardProjection = forwardProjectionPtr[index];
      tmp_fwdProjMissingPoints = fwdProjMissingPointsPtr[index];      
#endif
      if (!(location.x == 0 && location.y == 0 && location.z == 0 && normal.r == 0 && normal.g == 0 && normal.b == 0))
        {
		  ofs_sceneState << index << " ";
          ofs_sceneState << (int)rendering.x << " ";	// this is a Vector4 with all values equal;
          ofs_sceneState << location.x << " ";
          ofs_sceneState << location.y << " ";
          ofs_sceneState << location.z << " ";
          ofs_sceneState << normal.r << " ";
          ofs_sceneState << normal.g << " ";
          ofs_sceneState << normal.b << " ";
          ofs_sceneState << minmax.x << " " << minmax.y << " ";

          ofs_sceneState << tmp_raycastResult.x << " " << tmp_raycastResult.y << " ";
          ofs_sceneState << tmp_raycastResult.z << " " << tmp_raycastResult.w << " ";
          ofs_sceneState << tmp_forwardProjection.x << " " << tmp_forwardProjection.y << " ";
          ofs_sceneState << tmp_forwardProjection.z << " " << tmp_forwardProjection.w << " ";
          ofs_sceneState << tmp_fwdProjMissingPoints << std::endl;
        }
    }

  // print some values regarding the hashTable and the localVBA
  ofs_sceneState << scene->localVBA.lastFreeBlockId << " ";
  ofs_sceneState << scene->index.GetLastFreeExcessListId() << std::endl;

  // print entriesVisibleType
  const uchar *entriesVisibleType = renderingState_vh->GetEntriesVisibleType();

  for (int i = 0; i < numEntries; i++)
    {// loop over all the entries
      uchar entryVisibleType;
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&entryVisibleType, &entriesVisibleType[i], sizeof(uchar), cudaMemcpyDeviceToHost));
#else
      entryVisibleType = entriesVisibleType[i];
#endif

      if (entryVisibleType == 0)
        {
          ofs_sceneState << i << " 0" << std::endl;
        }
      else if (entryVisibleType == 1)
        {
          ofs_sceneState << i << " 1" << std::endl;
        }
      else if (entryVisibleType == 2)
        {
          ofs_sceneState << i << " 2" << std::endl;
        }
      else if (entryVisibleType == 3)
        {
          ofs_sceneState << i << " 3" << std::endl;
        }
    }

  // print the visible entries IDs
  ofs_sceneState << renderingState_vh->noVisibleEntries << std::endl;
  const int *visibleEntryIDs = renderingState_vh->GetVisibleEntryIDs();

  for (int i = 0; i < renderingState_vh->noVisibleEntries; i++)
  {
      int visibleEntryID;
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&visibleEntryID, &visibleEntryIDs[i], sizeof(int), cudaMemcpyDeviceToHost));
#else
	  visibleEntryID = visibleEntryIDs[i];
#endif
	  ofs_sceneState << visibleEntryID << std::endl;
  }

  // print the local VBA (allocationList) and hash Table (excessAllocationList)
  const int *allocationList = scene->localVBA.GetAllocationList();	
  for(int i=0;i<scene->index.getNumAllocatedVoxelBlocks();i++)
  {
    int allocationListEntry;
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&allocationListEntry, &allocationList[i], sizeof(int), cudaMemcpyDeviceToHost));
#else
	  allocationListEntry = allocationList[i];
#endif

    if(i!=scene->index.getNumAllocatedVoxelBlocks()-1)
    {
      ofs_sceneState << allocationListEntry << " ";
    }
    else
    {
      ofs_sceneState << allocationListEntry << std::endl;
    }
  }

  const int *excessAllocationList = scene->index.GetExcessAllocationList();	
  for(int i=0;i<SDF_EXCESS_LIST_SIZE;i++)
  {
    int excessAllocationListEntry;
#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&excessAllocationListEntry, &excessAllocationList[i], sizeof(int), cudaMemcpyDeviceToHost));
#else
	  excessAllocationListEntry = excessAllocationList[i];
#endif

    if(i!=SDF_EXCESS_LIST_SIZE-1)
    {
      ofs_sceneState << excessAllocationListEntry << " ";
    }
    else
    {
      ofs_sceneState << excessAllocationListEntry << std::endl;
    }
  }

  // print the local VBA and hash Table
  const ITMVoxel *localVBA = scene->localVBA.GetVoxelBlocks();	// pointer to the local voxel blocks array
  const ITMHashEntry *hashTable = scene->index.GetEntries();	// get pointer to the hashtable

  for (int i = 0; i < numEntries; i++)
    {// loop over all the entries
      // create a pointer to a ITMHashEntry object to store the retrieved entry from the hash table
      ITMHashEntry *hashEntry = new ITMHashEntry();
      // copy the corresponding entry from the hash table into the above object
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(hashEntry, &hashTable[i], sizeof(ITMHashEntry), cudaMemcpyDeviceToHost));
#else
      memcpy(hashEntry, &hashTable[i], sizeof(ITMHashEntry));
#endif
	  
      // // check validity of returned pointer
      if (!hashEntry->ptr || hashEntry->ptr < 0) continue;

      // get global position of the voxel block (one corner of the voxel block)
      Vector3i globalPosOfVoxelBlock;
      globalPosOfVoxelBlock.x = hashEntry->pos.x;
      globalPosOfVoxelBlock.y = hashEntry->pos.y;
      globalPosOfVoxelBlock.z = hashEntry->pos.z;
      //print the corner of the voxel block
	  ofs_sceneState << hashEntry->ptr << " ";
	  ofs_sceneState << hashEntry->offset << " ";
	  ofs_sceneState << globalPosOfVoxelBlock.x << " ";
      ofs_sceneState << globalPosOfVoxelBlock.y << " ";
      ofs_sceneState << globalPosOfVoxelBlock.z << " " << i << std::endl; // add the valid hash entry position in the hash table for saving

      globalPosOfVoxelBlock *= SDF_BLOCK_SIZE;

      // create a pointer to a ITMVoxel object to store the retrieved voxel block
      ITMVoxel *localVoxelBlock = (ITMVoxel*)malloc(sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
      // using the global position computed above, retrieve the voxel block from the localVBA and store it in the above object
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3), cudaMemcpyDeviceToHost));
#else
      memcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
#endif

      // loop over the voxel block to access each voxel within
      for (int z = 0; z < SDF_BLOCK_SIZE; z++) for (int y = 0; y < SDF_BLOCK_SIZE; y++) for (int x = 0; x < SDF_BLOCK_SIZE; x++)
        {
          // get linear index into the voxel block
          int locId = x + y * SDF_BLOCK_SIZE + z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

          // get position of the voxels
          // (these are in voxel units. to get values in metres multiply by voxelSize)
          Vector3f voxelPos;
          voxelPos.x = (float)(globalPosOfVoxelBlock.x + x);
          voxelPos.y = (float)(globalPosOfVoxelBlock.y + y);
          voxelPos.z = (float)(globalPosOfVoxelBlock.z + z);

          if (ITMVoxel::hasColorInformation)
            {
              // print the voxel position, its colour and SDF
              ofs_sceneState << (int)localVoxelBlock[locId].clr.r << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].clr.g << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].clr.b << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].w_color << " ";
              ofs_sceneState << localVoxelBlock[locId].sdf << " ";
              ofs_sceneState << (int)localVoxelBlock[locId].w_depth << std::endl;
              // ofs_sceneState << (int)localVoxelBlock[locId].w_depth << " ";
              // ofs_sceneState << localVoxelBlock[locId].last_update_time << std::endl;
            }
        }

    }

  ofs_sceneState.close();
  std::cout << "scene save to " << fileName << std::endl;
}

/** \brief
Loads the state of the scene, tracking state and rendering state from a saved file sceneState_<frameNo>_HT.txt.
\param[in]		frameNo							Current frameNo
\param[out]  scene							Pointer to an ITMScene object
\param[out]  trackingState					Pointer to an ITMTrackingState object
\param[out]  renderingState					Pointer to an ITMRenderingState object
\param[out]  hasStartedObjectReconstruction	Bool indicating if object reconstruction has started (not used!)
*/
template<class TVoxel>
void UtilsForVis<TVoxel, ITMVoxelBlockHash>::LoadCurrentState_CompleteInfo(const char *fileName, ITMScene<TVoxel, ITMVoxelBlockHash> *scene, ITMTrackingState *trackingState, ITMRenderState *renderingState, bool *hasStartedObjectReconstruction)
{
  std::ifstream ifs_sceneState;
  ifs_sceneState.open(fileName, std::ios::in);

  ITMRenderState_VH* renderingState_vh = (ITMRenderState_VH*)renderingState;

  // load the current pose
  std::string line;
  std::getline(ifs_sceneState, line);
  std::stringstream ss(line);
  Vector3f translation;
  Vector3f rotation;
  ss >> translation.x >> translation.y >> translation.z;
  ss >> rotation.x >> rotation.y >> rotation.z;
  trackingState->pose_d->SetFrom(translation, rotation);


  // load the pose_pointCloud
  std::getline(ifs_sceneState, line);
  std::stringstream ss_pc(line);
  ss_pc >> translation.x >> translation.y >> translation.z;
  ss_pc >> rotation.x >> rotation.y >> rotation.z;
  trackingState->pose_pointCloud->SetFrom(translation, rotation);
  std::cout<<"pose_pointCloud loaded~~~"<<std::endl;

  // load the age_pointCloud & requiresFullRendering in trackingState  #########################################################
  std::getline(ifs_sceneState, line);
  std::stringstream ss_age_pointCloud(line);
  ss_age_pointCloud >> trackingState->age_pointCloud;
  std::getline(ifs_sceneState, line);
  std::stringstream ss_requiresFullRendering(line);
  ss_requiresFullRendering >> trackingState->requiresFullRendering;
  std::getline(ifs_sceneState, line);
  std::stringstream ss_noFwdProjMissingPoints(line);
  ss_noFwdProjMissingPoints >> renderingState_vh->noFwdProjMissingPoints;
  

  MemoryDeviceType useGPU;
#ifndef COMPILE_WITHOUT_CUDA
  useGPU = MEMORYDEVICE_CUDA;
#else
  useGPU = MEMORYDEVICE_CPU;
#endif
  // load the other stuff in the trackingState and rendering state (in CPU)
  Vector4f *pointCloudLocations = trackingState->pointCloud->locations->GetData(useGPU);
  Vector4f *pointCloudNormals = trackingState->pointCloud->colours->GetData(useGPU);
  Vector4u *outRendering = renderingState_vh->raycastImage->GetData(useGPU);
  Vector2f *minMaxRange = renderingState_vh->renderingRangeImage->GetData(useGPU);
  Vector4f *raycastResultPtr = renderingState_vh->raycastResult->GetData(useGPU);
  Vector4f *forwardProjectionPtr = renderingState_vh->forwardProjection->GetData(useGPU);
  int *fwdProjMissingPointsPtr = renderingState_vh->fwdProjMissingPoints->GetData(useGPU);

  while (std::getline(ifs_sceneState, line))
    {
      Vector4f location, normal;
      Vector4u rendering;
      Vector2f minmax;
      Vector4f tmp_raycastResult;
      Vector4f tmp_forwardProjection;
      int tmp_fwdProjMissingPoints;      

      int index, render;
      std::stringstream ss1(line);

      if (!(ss1 >> index >> render >> location.x >> location.y >> location.z >> normal.r >> normal.g >> normal.b >> minmax.x >> minmax.y >> tmp_raycastResult.x >> tmp_raycastResult.y >> tmp_raycastResult.z >> tmp_raycastResult.w >> tmp_forwardProjection.x >> tmp_forwardProjection.y >> tmp_forwardProjection.z >> tmp_forwardProjection.w >> tmp_fwdProjMissingPoints))
        {
          // load the values of lastFreeBlockId and lastFreeExcessListId
          scene->localVBA.lastFreeBlockId = index;
		  scene->index.SetLastFreeExcessListId(render);
          break;
        }

      location.w = 0; normal.w = 0;

      rendering.x = (uchar)render;
      rendering.y = (uchar)render;
      rendering.z = (uchar)render;
      rendering.w = (uchar)render;
  
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&pointCloudLocations[index], &location, sizeof(Vector4f), cudaMemcpyHostToDevice));
      // ITMSafeCall(cudaMemcpy(pointCloudLocations[index], location, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&pointCloudNormals[index], &normal, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&outRendering[index], &rendering, sizeof(Vector4u), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&minMaxRange[index], &minmax, sizeof(Vector2f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&raycastResultPtr[index], &tmp_raycastResult, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&forwardProjectionPtr[index], &tmp_forwardProjection, sizeof(Vector4f), cudaMemcpyHostToDevice));
      ITMSafeCall(cudaMemcpy(&fwdProjMissingPointsPtr[index], &tmp_fwdProjMissingPoints, sizeof(int), cudaMemcpyHostToDevice));
#else
      // // memcpy(&pointCloudLocations[index], &location, sizeof(Vector4f));
      // memcpy(pointCloudLocations[index], location, sizeof(Vector4f));
      // memcpy(pointCloudNormals[index], normal, sizeof(Vector4f));
      // memcpy(outRendering[index], rendering, sizeof(Vector4u));
      // memcpy(minMaxRange[index], minmax, sizeof(Vector2f));
      // memcpy(raycastResultPtr[index], tmp_raycastResult, sizeof(Vector4f));
      // memcpy(forwardProjectionPtr[index], tmp_forwardProjection, sizeof(Vector4f));
      // memcpy(&fwdProjMissingPointsPtr[index], &tmp_fwdProjMissingPoints, sizeof(int));  
      pointCloudLocations[index] = location;
      pointCloudNormals[index] = normal;
      outRendering[index] = rendering;
      minMaxRange[index] = minmax;
      raycastResultPtr[index] = tmp_raycastResult;
      forwardProjectionPtr[index] = tmp_forwardProjection;
      fwdProjMissingPointsPtr[index] = tmp_fwdProjMissingPoints;
#endif  
    }

  #ifndef COMPILE_WITHOUT_CUDA
    trackingState->pointCloud->locations->UpdateHostFromDevice();
    trackingState->pointCloud->colours->UpdateHostFromDevice();
    renderingState_vh->raycastImage->UpdateHostFromDevice();
    renderingState_vh->renderingRangeImage->UpdateHostFromDevice();

    renderingState_vh->raycastResult->UpdateHostFromDevice();
    renderingState_vh->forwardProjection->UpdateHostFromDevice();
    renderingState_vh->fwdProjMissingPoints->UpdateHostFromDevice();  
  #endif


  // load the values of entries visible type
  uchar *entriesVisibleType = renderingState_vh->GetEntriesVisibleType();
  while (std::getline(ifs_sceneState, line))
    {
      int index;
      int val;
      std::stringstream ss2(line);

      if (!(ss2 >> index >> val))
        {// load the number of live indexIds
		  renderingState_vh->noVisibleEntries = index;
          break;
        }
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&entriesVisibleType[index], &val, sizeof(uchar), cudaMemcpyHostToDevice));
#else
      entriesVisibleType[index] = val;
#endif
    }

std::cout << "live entries: " << renderingState_vh->noVisibleEntries << std::endl;

  // load the values of live entry ids
int *visibleEntryIDs = renderingState_vh->GetVisibleEntryIDs();
int visibleEntryID;
  for (int i = 0; i < renderingState_vh->noVisibleEntries; i++)
    {
      std::getline(ifs_sceneState, line);
	  visibleEntryID = atoi(line.c_str());

#ifndef COMPILE_WITHOUT_CUDA
	  ITMSafeCall(cudaMemcpy(&visibleEntryIDs[i], &visibleEntryID, sizeof(int), cudaMemcpyHostToDevice));
#else
	  visibleEntryIDs[i] = visibleEntryID;
#endif

    }
std::cout << "Contents of live entries: " << std::endl;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  int *allocationList = scene->localVBA.GetAllocationList();
  std::getline(ifs_sceneState, line);
  std::stringstream ss9(line);
  int *tmpAllocationList = (int*) malloc(sizeof(int));
  for(int i=0;i<scene->index.getNumAllocatedVoxelBlocks();i++)
  {
    ss9 >> *tmpAllocationList;
#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&(allocationList[i]), tmpAllocationList, sizeof(int), cudaMemcpyHostToDevice));
#else
    memcpy(&(allocationList[i]), tmpAllocationList, sizeof(int));
#endif
  }

  int *excessAllocationList = scene->index.GetExcessAllocationList();
  std::getline(ifs_sceneState, line);
  std::stringstream ss10(line);
  int *tmpExcessAllocationList = (int*) malloc(sizeof(int));
  for(int i=0;i<SDF_EXCESS_LIST_SIZE;i++)
  {
    ss10 >> *tmpExcessAllocationList;
#ifndef COMPILE_WITHOUT_CUDA
    ITMSafeCall(cudaMemcpy(&(excessAllocationList[i]), tmpExcessAllocationList, sizeof(int), cudaMemcpyHostToDevice));
#else
    memcpy(&(excessAllocationList[i]), tmpExcessAllocationList, sizeof(int));
#endif
  }
// // Put outside loop -> buffer overflow error! 
//         // std::cout<<"BEFORE! fill allocationList "<<std::endl;
// #ifndef COMPILE_WITHOUT_CUDA
//       ITMSafeCall(cudaMemcpy(&(allocationList), tmpAllocationList, sizeof(int)* (scene->index.getNumAllocatedVoxelBlocks()), cudaMemcpyHostToDevice));
// #else
//       memcpy(&(allocationList), tmpAllocationList, sizeof(int)* (scene->index.getNumAllocatedVoxelBlocks()));
// #endif
//       // std::cout<<"AFTER! fill allocationList "<<std::endl;
//       // std::cout<<"BEFORE! fill excessAllocationList"<<std::endl;
// #ifndef COMPILE_WITHOUT_CUDA
//       ITMSafeCall(cudaMemcpy(&(excessAllocationList), tmpExcessAllocationList, sizeof(int)* (SDF_EXCESS_LIST_SIZE), cudaMemcpyHostToDevice));
// #else
//       memcpy(&(excessAllocationList), tmpExcessAllocationList, sizeof(int)* (SDF_EXCESS_LIST_SIZE));
// #endif
//       // std::cout<<"AFTER! fill excessAllocationList"<<std::endl;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~// 

  // load the local VBA and the hash table
  ITMVoxel *localVBA = scene->localVBA.GetVoxelBlocks();
  ITMHashEntry *hashTable = scene->index.GetEntries();
  const int numEntries = scene->index.noTotalEntries;	// total number of voxel blocks

  for (int i = 0; i < numEntries; i++)	// this loop actually goes only over number of valid entries
    {
      // create a new hash table entry
      ITMHashEntry *hashEntry = new ITMHashEntry();
      std::getline(ifs_sceneState, line);
      std::stringstream ss3(line);
      int hashEntryIdx;
      if (!(ss3 >> hashEntry->ptr >> hashEntry->offset >> hashEntry->pos.x >> hashEntry->pos.y >> hashEntry->pos.z >> hashEntryIdx))
        {
          std::cout<<"BREAK! load localVBA and HashTable Loop --- Entry "<<i<<"/"<<numEntries<<std::endl;
          break;
        }

      // localVoxelBlock that contains a block of 8x8x8 voxels. this will be pushed into the localVBA
      ITMVoxel *localVoxelBlock = (ITMVoxel*)malloc(sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));

      for (int locind = 0; locind < SDF_BLOCK_SIZE3; locind++)
        {
          int r, g, b, w_color, w_depth;
          short sdf;
          // float last_update_time;
          std::getline(ifs_sceneState, line);
          std::stringstream ss4(line);

          ss4 >> r >> g >> b >> w_color >> sdf >> w_depth;// >> last_update_time;
          // fill the local 8x8x8 voxel block
          localVoxelBlock[locind].clr.r = (uchar)r;
          localVoxelBlock[locind].clr.g = (uchar)g;
          localVoxelBlock[locind].clr.b = (uchar)b;
          localVoxelBlock[locind].w_color = (uchar)w_color;
          localVoxelBlock[locind].sdf = sdf;
          localVoxelBlock[locind].w_depth = (uchar)w_depth;
          // localVoxelBlock[locind].last_update_time = last_update_time;
        }

      // fill the local VBA with this block
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), localVoxelBlock, sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3), cudaMemcpyHostToDevice));
#else
      memcpy(&(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), localVoxelBlock, sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
#endif

      // fill hash table with the above hash entry
#ifndef COMPILE_WITHOUT_CUDA
      ITMSafeCall(cudaMemcpy(&hashTable[hashEntryIdx], hashEntry, sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
      // ITMSafeCall(cudaMemcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
#else
      memcpy(&hashTable[hashEntryIdx], hashEntry, sizeof(ITMHashEntry));
      // memcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry));
#endif
    }

  ifs_sceneState.close();
  std::cout << "loading from " << fileName << " is done :)" << std::endl;
}