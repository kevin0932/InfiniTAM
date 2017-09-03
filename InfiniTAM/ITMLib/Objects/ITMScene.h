// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"

#include "ITMSceneParams.h"
#include "ITMLocalVBA.h"
#include "ITMGlobalCache.h"



///////////////////////////////////////////////////////////////////////
#include <map>
// #include "../ITMLib.h"
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#include <boost/algorithm/string.hpp>

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

			~ITMScene(void)
			{
				if (useSwapping) delete globalCache;
			}

			// Suppress the default copy constructor and assignment operator
			ITMScene(const ITMScene&);
			ITMScene& operator=(const ITMScene&);


			//////////////////////////////////////////////////////////////////////////////
			////////// 		     Store globalCache	  	 /////////////////
			void writeGlobalCache(char *fileName)
			{
				std::cout<< "useSwapping= " << useSwapping << std::endl;
				globalCache->SaveToBinFile(fileName, sceneParams);
			}
			/////////////////////////////////////////////////////////////////////
			// Save all tsdf values to a local file (works with CUDA or CPU, and it is compatible with program enabling "useSwapping=True")
			void SaveTSDF2BinFile(const char *filePath, bool useBinary=false)
			{
				std::cout << "Saving TSDF of scene~~~~~~~~~~~~~" << std::endl;
				std::cout << "useSwapping = " << useSwapping << std::endl;
				const ITMHashEntry *hashTable = this->index.GetEntries();    // get pointer to the hash table (array of hash entries)
				const int numEntries = this->index.noTotalEntries;      // total number of voxel blocks, both allocated and non-allocated (should be 9 * 2^17 for default settings in libDefines.h)
				const ITMVoxel *localVBA = this->localVBA.GetVoxelBlocks();  // pointer to the local voxel blocks array

				int voxelBlocksX = 2048;             // number of voxel blocks along X
				int voxelBlocksY = 2048;             // number of voxel blocks along Y
				int voxelBlocksZ = 2048;             // number of voxel blocks along Z
				int voxelOffsetX = voxelBlocksX / 2;   // half of above value
				int voxelOffsetY = voxelBlocksY / 2;   // half of above value
				int voxelOffsetZ = voxelBlocksZ / 2;   // half of above value

				std::map<long long unsigned int, short> mapIndexAndTSDF_Storage;
				std::map<long long unsigned int, short> mapIndexAndTSDF;
				std::map<long long unsigned int, short> mapIndexAndTSDF_GlobalCache;
				std::map<long long unsigned int, Vector3i > mapIndexAndPos;
				std::map<long long unsigned int, Vector3u > mapIndexAndColor;

				int nValidEntries_LocalBVA = 0;
				int nValidEntries_GlobalCache = 0;
				long long unsigned int nVoxels_LocalBVA = 0;
				long long unsigned int nVoxels_GlobalCache = 0;				

				for (int i = 0; i < numEntries; i++)
				{
					// a true/false flag indicating where the current voxel block should be retrieved (GlobalCache/LocalVBA)
					bool inGlobalCache = false;	

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

					if (hashEntry->ptr < 0 && hashEntry->ptr != -1)
					{
						continue;
					}
						
					if (hashEntry->ptr != -1) //hashEntry->ptr == -1 indicates that data has been swapped out of localVBA
					{
						nValidEntries_LocalBVA++;
					} 
					else	// therefore, retrieve data from GlobalCache, instead of localVBA
					{
						nValidEntries_GlobalCache++;
						inGlobalCache = true;
					}

					ITMVoxel *localVoxelBlock = (ITMVoxel*)malloc(sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
					// ITMVoxel *globalVoxelBlock = (ITMVoxel*)malloc(sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));

					// get global position of voxel block (one corner of the voxel block)
					Vector3i globalPosOfVoxelBlock;
					globalPosOfVoxelBlock.x = hashEntry->pos.x;
					globalPosOfVoxelBlock.y = hashEntry->pos.y;
					globalPosOfVoxelBlock.z = hashEntry->pos.z;
					globalPosOfVoxelBlock *= SDF_BLOCK_SIZE;
													
					if(inGlobalCache == false)
					{
						// using the global position computed above, retrieve the voxel block from the globalVBA and store it in the above object
					#ifndef COMPILE_WITHOUT_CUDA
						ITMSafeCall(cudaMemcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3), cudaMemcpyDeviceToHost));
					#else
						memcpy(localVoxelBlock, &(localVBA[hashEntry->ptr * (SDF_BLOCK_SIZE3)]), sizeof(ITMVoxel)* (SDF_BLOCK_SIZE3));
					#endif
					}

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
								// should be check to save memory storage since 32767(short) or 1(tsdf value) indicates empty voxels!
								// if(localVoxelBlock[locId].sdf != 32767) 
								{
									// push only those voxels that were allocated
									long long unsigned int linIndex = (vIndex.x + voxelOffsetX) + (vIndex.y + voxelOffsetY) * voxelBlocksX + (vIndex.z + voxelOffsetZ) * voxelBlocksY * voxelBlocksX;
									
									if( inGlobalCache == true && globalCache->GetStoredVoxelBlock(i)[locId].sdf != 32767 && useSwapping == true )
									{
										mapIndexAndTSDF_Storage.insert(std::make_pair(linIndex, globalCache->GetStoredVoxelBlock(i)[locId].sdf));
										mapIndexAndPos.insert(std::make_pair(linIndex, vIndex));
										mapIndexAndColor.insert(std::make_pair(linIndex, localVoxelBlock[locId].clr));

										nVoxels_GlobalCache++;
									}
									if( inGlobalCache == false && localVoxelBlock[locId].sdf != 32767 )
									{
										mapIndexAndTSDF_Storage.insert(std::make_pair(linIndex, localVoxelBlock[locId].sdf));
										mapIndexAndPos.insert(std::make_pair(linIndex, vIndex));
										mapIndexAndColor.insert(std::make_pair(linIndex, localVoxelBlock[locId].clr));

										nVoxels_LocalBVA++;
									}
									// std::cout<<"local sdf = "<<localVoxelBlock[locId].sdf<<"; global sdf = "<<globalCache->GetStoredVoxelBlock(i)[locId].sdf<<std::endl;
								}
							} //<-- end of loop over x
						} //<-- end of loop over y
					} //<-- end of loop over z

				} //<-- end of loop over numEntries

				std::cout << "Number of allocated voxel blocks (local):" << nValidEntries_LocalBVA << std::endl;
				std::cout << "Number of allocated voxel blocks (global):" << nValidEntries_GlobalCache << std::endl;
				std::cout << "Number of allocated voxels (local):" << nVoxels_LocalBVA << std::endl;
				std::cout << "Number of allocated voxels (global):" << nVoxels_GlobalCache << std::endl;				
				long long unsigned int numTotalVoxels = nVoxels_LocalBVA+nVoxels_GlobalCache;

				// Save the tsdf values for storage
				if(useBinary==false)
				{
					FILE *f = fopen(filePath, "w");
					fprintf(f, "%llu", numTotalVoxels);
					fprintf(f, "\n");
					std::map<long long unsigned int, short>::iterator it;
					for (it = mapIndexAndTSDF_Storage.begin(); it != mapIndexAndTSDF_Storage.end(); it++)
					{
						// write all tsdf (position+tsdf) to file
						Vector3i pos = mapIndexAndPos[it->first];     // compute back indices
						short tsdfVal = mapIndexAndTSDF_Storage[it->first];
						Vector3u colorVox = mapIndexAndColor[it->first];
						fprintf(f, "%i %i %i", pos.x, pos.y, pos.z);    // write position to file
						fprintf(f, " %i", tsdfVal);	// write tsdf of current voxel to file
						fprintf(f, " %u %u %u", colorVox.x, colorVox.y, colorVox.z); // write color of current voxel to file if it is available
						fprintf(f, "\n");
					}
					fclose(f);
				}
				else
				{
					// // Get the fileName without .txt extension and make up fileName for .bin file
					// std::vector<std::string> tmpStr;
					// boost::split(tmpStr, filePath, [](char c){return c == '.';});

					// char binFileName[250];
					// std::cout<<"extension change debug: "<< tmpStr[0] << std::endl;
					// strcpy(binFileName, tmpStr[0].c_str());
					// strcat(binFileName, ".bin");
					// std::cout<<"extension change debug: "<< binFileName << std::endl;

					// Save the tsdf values for storage (.bin)
					FILE *fbin = fopen("tsdf_saved.bin", "wb");
					fwrite(&numTotalVoxels, sizeof(long long unsigned int), 1, fbin);
					std::map<long long unsigned int, short>::iterator itBin;
					for (itBin = mapIndexAndTSDF_Storage.begin(); itBin != mapIndexAndTSDF_Storage.end(); itBin++)
					{
						// write all tsdf (position+tsdf) to file
						Vector3i pos = mapIndexAndPos[itBin->first];     // compute back indices
						short tsdfVal = mapIndexAndTSDF_Storage[itBin->first];
						Vector3u colorVox = mapIndexAndColor[itBin->first];

						fwrite(&pos, sizeof(Vector3i), 1, fbin);
						fwrite(&tsdfVal, sizeof(short), 1, fbin);
						fwrite(&colorVox, sizeof(Vector3u), 1, fbin);
					}
					fclose(fbin);
				}

				std::cout << "writing tsdf file --- Done!" << std::endl;
				return;
			}

			// Save all hash entries to a local file (works with CUDA or CPU)
			void SaveHashTable2File(const char *filePath, bool useBinary=false)
			{
				std::cout << "Saving HashTable of scene~~~~~~~~~~~~~" << std::endl;
				const ITMHashEntry *hashTable = this->index.GetEntries();    // get pointer to the hash table (array of hash entries)
				const int numEntries = this->index.noTotalEntries;      // total number of voxel blocks, both allocated and non-allocated (should be 9 * 2^17 for default settings in libDefines.h)
				const ITMVoxel *localVBA = this->localVBA.GetVoxelBlocks();  // pointer to the local voxel blocks array

				if(useBinary==false)
				{
					FILE *f = fopen(filePath, "w");
					fprintf(f, "%llu", numEntries);
					fprintf(f, "\n");

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
						// fprintf(f, "%i %i %i", hashEntry->pos.x, hashEntry->pos.y, hashEntry->pos.z);    // write position to file
						fprintf(f, "%hi %hi %hi", hashEntry->pos.x, hashEntry->pos.y, hashEntry->pos.z);    // write position to file
						fprintf(f, " %i %i", hashEntry->ptr, hashEntry->offset);
						fprintf(f, "\n");
					}

					fclose(f);
				}
				else
				{
					// // Get the fileName without .txt extension and make up fileName for .bin file
					// std::vector<std::string> tmpStr;
					// boost::split(tmpStr, filePath, [](char c){return c == '.';});

					// char binFileName[250];
					// strcpy(binFileName, tmpStr[0].c_str());
					// strcat(binFileName, ".bin");

					FILE *fbin = fopen("hashtable_saved.bin", "wb");
					fwrite(&numEntries, sizeof(long long unsigned int), 1, fbin);

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

						fwrite(hashEntry, sizeof(ITMHashEntry), 1, fbin);
						// fwrite(&hashEntry->pos, sizeof(Vector3s), 1, fbin);
						// fwrite(&hashEntry->ptr, sizeof(int), 1, fbin);
						// fwrite(&hashEntry->offset, sizeof(int), 1, fbin);
					}

					fclose(fbin);
				}

				std::cout << "writing hash table --- Done!" << std::endl;
				return;
			}

			// Load all hash entries from a local file (works with CUDA or CPU)
			// It will first clear the hash table to default values and then reload pre-saved hash table from the previous session
			void LoadHashTableFromFile(const char *filePath, bool useBinary=false)
			{
				std::cout << "Load HashTable of scene~~~~~~~~~~~~~" << std::endl;
				ITMHashEntry *hashTable = this->index.GetEntries();    // get pointer to the hash table (array of hash entries)
				const int numEntries = this->index.noTotalEntries;      // total number of voxel blocks, both allocated and non-allocated (should be 9 * 2^17 for default settings in libDefines.h)

				int HashTableSize = 0;

				if(useBinary==false)
				{
					std::ifstream ifs_HashTable;
					std::string line;					
					ifs_HashTable.open(filePath, std::ios::in);	
					std::getline(ifs_HashTable, line);
					std::stringstream ss_HashTableSize(line);
					ss_HashTableSize >> HashTableSize;

					// // clear/reset HashTable
					this->index.ClearEntries();

					// ITMHashEntry *InitHashEntry = new ITMHashEntry();
					// for (int i = 0; i < numEntries; i++)
					// {
					// 	// hashTable[i] = *InitHashEntry;
					// 	#ifndef COMPILE_WITHOUT_CUDA
					// 		ITMSafeCall(cudaMemcpy(&hashTable[i], InitHashEntry, sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
					// 	#else
					// 		// memcpy(&hashTable[i], InitHashEntry, sizeof(ITMHashEntry));
					// 		hashTable[i] = *InitHashEntry;
					// 	#endif
					// }

					for (int i = 0; i < numEntries; i++)
					{
						// create a new hash table entry
						ITMHashEntry *hashEntry = new ITMHashEntry();
						std::getline(ifs_HashTable, line);
						std::stringstream ss3(line);
						if (!(ss3 >> hashEntry->pos.x >> hashEntry->pos.y >> hashEntry->pos.z >> hashEntry->ptr >> hashEntry->offset))
						{
							std::cout<<"BREAK! load localVBA and HashTable Loop --- Entry "<<i<<"/"<<numEntries<<std::endl;
							break;
						}

						#ifndef COMPILE_WITHOUT_CUDA
							ITMSafeCall(cudaMemcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
						#else
							memcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry));
						#endif
					}
					ifs_HashTable.close();
				}
				else	// Bug from Loading Hash Table from binary file should be fixed!!!
				{
					// // Get the fileName without .txt extension and make up fileName for .bin file
					// std::vector<std::string> tmpStr;
					// boost::split(tmpStr, filePath, [](char c){return c == '.';});

					// char binFileName[250];
					// strcpy(binFileName, tmpStr[0].c_str());
					// strcat(binFileName, ".bin");

					// ifs_HashTable.open(filePath, std::ios::in | std::ios::binary);	
				    std::ifstream ifs_HashTable("hashtable_saved.bin", std::ios::binary | std::ios::in);
					ifs_HashTable.read(reinterpret_cast<char*>(&HashTableSize), sizeof(long long unsigned int));

					// // clear/reset HashTable
					this->index.ClearEntries();

					for (int i = 0; i < numEntries; i++)
					{
						// create a new hash table entry
						ITMHashEntry *hashEntry = new ITMHashEntry();

    					ifs_HashTable.read(reinterpret_cast<char*>(hashEntry), sizeof(ITMHashEntry));
						// ifs_HashTable.read(reinterpret_cast<char*>(&(hashEntry->pos)), sizeof(Vector3s));
						// ifs_HashTable.read(reinterpret_cast<char*>(&(hashEntry->ptr)), sizeof(int));
						// ifs_HashTable.read(reinterpret_cast<char*>(&(hashEntry->offset)), sizeof(int));
					// std::cout<<"IO Debug: hashEntry->pos = "<<hashEntry->pos.x<<", "<<hashEntry->pos.y<<", "<<hashEntry->pos.z<<"; ptr = "<<hashEntry->ptr<<"; offset = "<<hashEntry->offset<<std::endl;

						#ifndef COMPILE_WITHOUT_CUDA
							ITMSafeCall(cudaMemcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry), cudaMemcpyHostToDevice));
						#else
							memcpy(&hashTable[i], hashEntry, sizeof(ITMHashEntry));
						#endif
					}
					ifs_HashTable.close();
				}

				std::cout << "HashTable loaded --- done" << std::endl;
				return;
			}
			//////////////////////////////////////////////////////////////////////////////
		};
	}
}