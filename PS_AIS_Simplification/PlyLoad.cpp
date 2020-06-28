#include "PlyLoad.h"

CPLYLoader::CPLYLoader()
{
	this->m_totalConnectedQuads = 0;
	this->m_totalConnectedPoints = 0;
	m_ModelData.iTotalConnectedTriangles = 0;
}

int CPLYLoader::LoadModel(char* filename)
{
	printf("Loading %s...\n", filename);
	char* pch = strstr(filename, ".ply");

	if (pch != NULL)
	{
		FILE* file = fopen(filename, "r");
		if (!file)
		{
			printf("load PLY file %s failed\n", filename);
			return false;
		}
		fseek(file, 0, SEEK_END);
		long fileSize = ftell(file);

		try
		{
			mp_vertexXYZ = (float*)malloc(ftell(file));
			mp_vertexNorm = (float*)malloc(ftell(file));
			mp_vertexRGB = (float*)malloc(ftell(file));
		}
		catch (char*)
		{
			return -1;
		}
		if (mp_vertexXYZ == NULL) return -1;
		if (mp_vertexNorm == NULL) return -2;
		if (mp_vertexRGB == NULL) return -3;
		fseek(file, 0, SEEK_SET);

		if (file)
		{
			int i = 0;
			int temp = 0;
			int quads_index = 0;
			int triangle_index = 0;
			int normal_index = 0;
			int colorIndex = 0;
			char buffer[1000];


			fgets(buffer, 300, file);			// ply


			// READ HEADER
			// -----------------

			// Find number of vertexes
			while (strncmp("element vertex", buffer, strlen("element vertex")) != 0)
			{
				fgets(buffer, 300, file);			// format
			}
			strcpy(buffer, buffer + strlen("element vertex"));
			sscanf(buffer, "%i", &this->m_totalConnectedPoints);


			// Find number of vertexes
			fseek(file, 0, SEEK_SET);
			while (strncmp("element face", buffer, strlen("element face")) != 0)
			{
				fgets(buffer, 300, file);			// format
			}
			strcpy(buffer, buffer + strlen("element face"));
			sscanf(buffer, "%i", &this->m_totalFaces);


			// go to end_header
			while (strncmp("end_header", buffer, strlen("end_header")) != 0)
			{
				fgets(buffer, 300, file);			// format
			}

			//----------------------


			// read vertices
			i = 0;
			for (int iterator = 0; iterator < this->m_totalConnectedPoints; iterator++)//3488
			{
				char tmp[1];
				fgets(buffer, 300, file);

				sscanf(buffer, "%f %f %f %f %f %f %c %f %f %f", &mp_vertexXYZ[i], &mp_vertexXYZ[i + 1], &mp_vertexXYZ[i + 2],
					&mp_vertexNorm[i], &mp_vertexNorm[i + 1], &mp_vertexNorm[i + 2],
					tmp,
					&mp_vertexRGB[i], &mp_vertexRGB[i + 1], &mp_vertexRGB[i + 2]);
				
				vector<double> points_i;
				vector<double> normals_i;
				vector<double> colors_i;
				points_i.push_back(mp_vertexXYZ[i]);
				points_i.push_back(mp_vertexXYZ[i + 1]);
				points_i.push_back(mp_vertexXYZ[i + 2]);
				normals_i.push_back(mp_vertexNorm[i]);
				normals_i.push_back(mp_vertexNorm[i + 1]);
				normals_i.push_back(mp_vertexNorm[i + 2]);
				colors_i.push_back(mp_vertexRGB[i]);
				colors_i.push_back(mp_vertexRGB[i + 1]);
				colors_i.push_back(mp_vertexRGB[i + 2]);
				points.push_back(points_i);
				normals.push_back(normals_i);
				colors.push_back(colors_i);
				i += 3;
			}

			// read faces
			i = 0;
			for (int iterator = 0; iterator < this->m_totalFaces; iterator++)//6920
			{
				fgets(buffer, 300, file);

				if (buffer[0] == '3')
				{
					int vertex1 = 0, vertex2 = 0, vertex3 = 0;
					buffer[0] = ' ';
					sscanf(buffer, "%i%i%i", &vertex1, &vertex2, &vertex3);//number of vertex eg:5,7,6

					//load the trangular edges
					vector<int> edges_tem;
					edges_tem.push_back(vertex1);
					edges_tem.push_back(vertex2);
					edges_tem.push_back(vertex3);
					vecFaceIndex.push_back(edges_tem);					

					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex1]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex1 + 1]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex1 + 2]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex2]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex2 + 1]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex2 + 2]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex3]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex3 + 1]);
					m_ModelData.vecFaceTriangles.push_back(mp_vertexXYZ[3 * vertex3 + 2]);


					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex1] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex1 + 1] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex1 + 2] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex2] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex2 + 1] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex2 + 2] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex3] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex3 + 1] / 255.0f);
					m_ModelData.vecFaceTriangleColors.push_back(mp_vertexRGB[3 * vertex3 + 2] / 255.0f);

					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex1]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex1 + 1]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex1 + 2]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex2]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex2 + 1]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex2 + 2]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex3]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex3 + 1]);
					m_ModelData.vecNormals.push_back(mp_vertexNorm[3 * vertex3 + 2]);

					triangle_index += 9;
					m_ModelData.iTotalConnectedTriangles += 3;
				}


				i += 3;
			}

			fclose(file);
			printf("%s Loaded!\n", filename);

		}

		else
		{
			printf("File can't be opened\n");
		}
	}
	else
	{
		printf("File does not have a .PLY extension. ");
	}

	return 0;
}

void CPLYLoader::Draw() //implemented in GLPainter, not called again
{
	if (m_ModelData.vecFaceTriangleColors.empty())
	{
		cout << "model data is null" << endl;
		exit(-1);
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, m_ModelData.vecFaceTriangles.data());
	glColorPointer(3, GL_FLOAT, 0, m_ModelData.vecFaceTriangleColors.data());
	glNormalPointer(GL_FLOAT, 0, m_ModelData.vecNormals.data());
	glDrawArrays(GL_TRIANGLES, 0, m_ModelData.iTotalConnectedTriangles);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}