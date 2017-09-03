// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "ITMSceneParams.h"
#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"


#include<iostream>

///////////////////////////////////////////////////////////////////////
#include <map>
// #include "../ITMLib.h"
#include <sstream>
#include <vector>
#include <fstream>
///////////////////////////////////////////////////////////////////////


namespace ITMLib
{
	namespace Objects
	{
		/** \brief
		Represents the 3D world model as a hash of small voxel
		blocks
		*/
		template<class TVoxel, class TIndex>
		class ITMScene
		{
		public:
			bool useSwapping;

			/** Scene parameters like voxel size etc. */
			const ITMSceneParams *sceneParams;

			/** Hash table to reference the 8x8x8 blocks */
			TIndex index;

			/** Current local content of the 8x8x8 voxel blocks -- stored host or device */
			ITMLocalVBA<TVoxel> localVBA;

			/** Global content of the 8x8x8 voxel blocks -- stored on host only */
			ITMGlobalCache<TVoxel> *globalCache;

			ITMScene(const ITMSceneParams *sceneParams, bool useSwapping, MemoryDeviceType memoryType)
				: index(memoryType), localVBA(memoryType, index.getNumAllocatedVoxelBlocks(), index.getVoxelBlockSize())
			{
				this->sceneParams = sceneParams;
				this->useSwapping = useSwapping;
				if (useSwapping) globalCache = new ITMGlobalCache<TVoxel>();
			}

			//////////////////////////////////////////////////////////////////////////////
			////////// 		     Store the scene from globalCache	  	 /////////////////
			void writeGlobalCache(char *fileName)
			{
				std::cout<< "useSwapping= " << useSwapping << std::endl;
				globalCache->SaveToBinFile(fileName, sceneParams);


				// ITMGlobalCache<TVoxel> *tmp_globalCache = this->globalCache;
				// this->globalCache.SaveToFile(fileName);
				// const TVoxel *localVBAtmp = this->localVBA.GetVoxelBlocks();
				// std::cout<<"*localVBAtmp: "<<localVBAtmp[0].sdf<<std::endl;
			}

			///////////////////////////////////////////////////////////////////////
			void RetrieveTSDFbyVoxelToBinFile(char *fileName) const
			{
				const ITMHashEntry *hashTable = this->index.GetEntries();    // get pointer to the hash table (array of hash entries)
				const int numEntries = this->index.noTotalEntries;      // total number of voxel blocks, both allocated and non-allocated (should be 9 * 2^17 for default settings in libDefines.h)
				// std::cout<<"numEntries = "<<numEntries<<std::endl;
				const ITMVoxel *localVBA = this->localVBA.GetVoxelBlocks();  // pointer to the local voxel blocks array	
				
			}
			///////////////////////////////////////////////////////////////////////
			// template<class TVoxel>
			void SaveTSDF2BinFile(const char *fileName)
			// void SaveTSDF2BinFile(const int frameNo)
			{
			std::cout << "Saving TSDF of scene. use marching cubes to create mesh" << std::endl;
			// const ITMHashEntry *hashTable = scene->index.GetEntries();    // get pointer to the hash table (array of hash entries)
			// const int numEntries = scene->index.noTotalEntries;      // total number of voxel blocks, both allocated and non-allocated (should be 9 * 2^17 for default settings in libDefines.h)
			// const ITMVoxel *localVBA = scene->localVBA.GetVoxelBlocks();  // pointer to the local voxel blocks array
			const ITMHashEntry *hashTable = this->index.GetEntries();    // get pointer to the hash table (array of hash entries)
			const int numEntries = this->index.noTotalEntries;      // total number of voxel blocks, both allocated and non-allocated (should be 9 * 2^17 for default settings in libDefines.h)
			// std::cout<<"numEntries = "<<numEntries<<std::endl;
			const ITMVoxel *localVBA = this->localVBA.GetVoxelBlocks();  // pointer to the local voxel blocks array			

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
			// ss_name << "tsdf_" << frameNo << ".txt";
			// FILE *f = fopen(ss_name.str().c_str(), "w");
			FILE *f = fopen(fileName, "w");
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

			//////////////////////////////////////////////////////////////////////////////

			~ITMScene(void)
			{
				if (useSwapping) delete globalCache;
			}

			// Suppress the default copy constructor and assignment operator
			ITMScene(const ITMScene&);
			ITMScene& operator=(const ITMScene&);
		};
	}
}