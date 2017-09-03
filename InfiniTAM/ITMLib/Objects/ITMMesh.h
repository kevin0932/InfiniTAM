// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/ITMLibDefines.h"
#include "../../ORUtils/Image.h"

#include <stdlib.h>

namespace ITMLib
{
	namespace Objects
	{
		class ITMMesh
		{
		public:
			struct Triangle { Vector3f p0, p1, p2; Vector3u c0, c1, c2; };
		
			MemoryDeviceType memoryType;

			uint noTotalTriangles;
			bool hasColour;
			static const uint noMaxTriangles = SDF_LOCAL_BLOCK_NUM * 32;

			ORUtils::MemoryBlock<Triangle> *triangles;

			explicit ITMMesh(MemoryDeviceType memoryType)
			{
				this->memoryType = memoryType;
				this->noTotalTriangles = 0;
				// this->hasColour = hasColourInformation;
				this->hasColour = false;
				
				triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, memoryType);
			}

			void WriteOBJ(const char *fileName)
			{
				ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
				if (memoryType == MEMORYDEVICE_CUDA)
				{
					cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
					cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
					shoulDelete = true;
				}
				else cpu_triangles = triangles;

				Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

				FILE *f = fopen(fileName, "w+");
				if (f != NULL)
				{
					for (uint i = 0; i < noTotalTriangles; i++)
					{
						fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
						fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
						fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
					}

					for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
					fclose(f);
				}

				if (shoulDelete) delete cpu_triangles;
			}

			void WriteSTL(const char *fileName)
			{
				ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
				if (memoryType == MEMORYDEVICE_CUDA)
				{
					cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
					cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
					shoulDelete = true;
				}
				else cpu_triangles = triangles;

				Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

				FILE *f = fopen(fileName, "wb+");

				if (f != NULL) {
					for (int i = 0; i < 80; i++) fwrite(" ", sizeof(char), 1, f);

					fwrite(&noTotalTriangles, sizeof(int), 1, f);

					float zero = 0.0f; short attribute = 0;
					for (uint i = 0; i < noTotalTriangles; i++)
					{
						fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f); fwrite(&zero, sizeof(float), 1, f);

						fwrite(&triangleArray[i].p2.x, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p2.y, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p2.z, sizeof(float), 1, f);

						fwrite(&triangleArray[i].p1.x, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p1.y, sizeof(float), 1, f); 
						fwrite(&triangleArray[i].p1.z, sizeof(float), 1, f);

						fwrite(&triangleArray[i].p0.x, sizeof(float), 1, f);
						fwrite(&triangleArray[i].p0.y, sizeof(float), 1, f);
						fwrite(&triangleArray[i].p0.z, sizeof(float), 1, f);

						fwrite(&attribute, sizeof(short), 1, f);

						//fprintf(f, "v %f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
						//fprintf(f, "v %f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
						//fprintf(f, "v %f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
					}

					//for (uint i = 0; i<noTotalTriangles; i++) fprintf(f, "f %d %d %d\n", i * 3 + 2 + 1, i * 3 + 1 + 1, i * 3 + 0 + 1);
					fclose(f);
				}

				if (shoulDelete) delete cpu_triangles;
			}


			void WritePLY(const char *fileName, bool is_binary=false)
			{
			  	ORUtils::MemoryBlock<Triangle> *cpu_triangles; bool shoulDelete = false;
				if (memoryType == MEMORYDEVICE_CUDA)
				{
				cpu_triangles = new ORUtils::MemoryBlock<Triangle>(noMaxTriangles, MEMORYDEVICE_CPU);
				cpu_triangles->SetFrom(triangles, ORUtils::MemoryBlock<Triangle>::CUDA_TO_CPU);
				shoulDelete = true;
				}
				else cpu_triangles = triangles;

				Triangle *triangleArray = cpu_triangles->GetData(MEMORYDEVICE_CPU);

				FILE *f;
				if (is_binary) {
				f = fopen(fileName, "wb+");
				} else {
				f = fopen(fileName, "w+");
				}
				if (f != NULL)
				{
				std::cout << "opened mesh file." << std::endl;
				fprintf(f, "ply\n");
				if (is_binary) {
					fprintf(f, "format binary_little_endian 1.0\n");
				} else {
					fprintf(f, "format ascii 1.0\n");
				}
				fprintf(f, "element vertex %d\n", noTotalTriangles*3);
				fprintf(f, "property float x\n");
				fprintf(f, "property float y\n");
				fprintf(f, "property float z\n");
				if (hasColour) {
					fprintf(f, "property uchar red\n");
					fprintf(f, "property uchar green\n");
					fprintf(f, "property uchar blue\n");
				}
				fprintf(f, "element face %d\n", noTotalTriangles);
				fprintf(f, "property list uchar int vertex_indices\n");
				fprintf(f, "end_header\n");

				for (uint i = 0; i < noTotalTriangles; i++)
				{
					if (abs(triangleArray[i].p0.x) > 100.0 || abs(triangleArray[i].p0.y) > 100.0 || abs(triangleArray[i].p0.z) > 100.0 ||
						abs(triangleArray[i].p1.x) > 100.0 || abs(triangleArray[i].p1.y) > 100.0 || abs(triangleArray[i].p1.z) > 100.0 ||
						abs(triangleArray[i].p2.x) > 100.0 || abs(triangleArray[i].p2.y) > 100.0 || abs(triangleArray[i].p2.z) > 100.0) {
					triangleArray[i].p0.x = 100.0;
					triangleArray[i].p0.y = 100.0;
					triangleArray[i].p0.z = 100.0;
					triangleArray[i].p1.x = 100.1;
					triangleArray[i].p1.y = 100.0;
					triangleArray[i].p1.z = 100.0;
					triangleArray[i].p2.x = 100.0;
					triangleArray[i].p2.y = 100.1;
					triangleArray[i].p2.z = 100.0;
					}
					if (hasColour) {
					if (is_binary) {
						fwrite(triangleArray[i].p0, sizeof(Vector3f), 1, f);
						fwrite(triangleArray[i].c0, sizeof(Vector3u), 1, f);
						fwrite(triangleArray[i].p1, sizeof(Vector3f), 1, f);
						fwrite(triangleArray[i].c1, sizeof(Vector3u), 1, f);
						fwrite(triangleArray[i].p2, sizeof(Vector3f), 1, f);
						fwrite(triangleArray[i].c2, sizeof(Vector3u), 1, f);
					} else {
						fprintf(f, "%f %f %f %u %u %u\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z, triangleArray[i].c0.r, triangleArray[i].c0.g, triangleArray[i].c0.b);
						fprintf(f, "%f %f %f %u %u %u\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z, triangleArray[i].c1.r, triangleArray[i].c1.g, triangleArray[i].c1.b);
						fprintf(f, "%f %f %f %u %u %u\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z, triangleArray[i].c2.r, triangleArray[i].c2.g, triangleArray[i].c2.b);
					}
					} else {
					if (is_binary) {
						fwrite(triangleArray[i].p0, sizeof(Vector3f), 1, f);
						fwrite(triangleArray[i].p1, sizeof(Vector3f), 1, f);
						fwrite(triangleArray[i].p2, sizeof(Vector3f), 1, f);
					} else {
						fprintf(f, "%f %f %f\n", triangleArray[i].p0.x, triangleArray[i].p0.y, triangleArray[i].p0.z);
						fprintf(f, "%f %f %f\n", triangleArray[i].p1.x, triangleArray[i].p1.y, triangleArray[i].p1.z);
						fprintf(f, "%f %f %f\n", triangleArray[i].p2.x, triangleArray[i].p2.y, triangleArray[i].p2.z);
					}
					}
				}

				// faces
				unsigned char sides = 3;
				for (uint i = 0; i<noTotalTriangles; i++) {
					if (is_binary) {
					Vector3i face(i * 3 + 2, i * 3 + 1, i * 3 + 0);
					fwrite(&sides, sizeof(unsigned char), 1, f);
					fwrite(face, sizeof(Vector3i), 1, f);
					} else {
					fprintf(f, "3 %d %d %d\n", i * 3 + 2, i * 3 + 1, i * 3 + 0);
					}
				}
				fclose(f);
				std::cout << "done writing to mesh file." << std::endl;
				}

				if (shoulDelete) delete cpu_triangles;
			}

			~ITMMesh()
			{
				delete triangles;
			}

			// Suppress the default copy constructor and assignment operator
			ITMMesh(const ITMMesh&);
			ITMMesh& operator=(const ITMMesh&);
		};
	}
}
