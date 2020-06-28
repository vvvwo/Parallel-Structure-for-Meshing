/*    
 *  GLM library.  Wavefront .obj file format reader/writer/manipulator.
 *
 *  Written by Nate Robins, 1997.
 *  email: ndr@pobox.com
 *  www: http://www.pobox.com/~ndr
 */

/* includes */
#pragma once
#include <cmath>
using namespace std;

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include "glm.h"
#include "glmVector.h"
#include <vector>

/* Some <math.h> files do not define M_PI... */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* defines */
#define T(x) model->triangles[(x)]

/* enums */
enum { X, Y, Z, W };      /* elements of a vertex */



/* global variables */
/*
GLubyte *_teximage_new;
static int _teximageWidth_new = 1, _teximageHeight_new = 1;
*/

#define MAX_TEXTURES 100
static GLuint textureArray[MAX_TEXTURES] = {0};

vector<int> boxCoordinate;//����ģ��box������

/* typedefs */

/* _GLMnode: general purpose node
 */
typedef struct _GLMnode {
  GLuint           index;
  GLboolean        averaged;
  struct _GLMnode* next;
} GLMnode;

/* strdup is actually not a standard ANSI C or POSIX routine
   so implement a private one.  OpenVMS does not have a strdup; Linux's
   standard libc doesn't declare strdup by default (unless BSD or SVID
   interfaces are requested). */
  static char *
stralloc(const char *string)
{
  char *copy;

  copy = (char*) malloc(strlen(string) + 1);
  if (copy == NULL)
    return NULL;
  strcpy(copy, string);
  return copy;
}

/* private functions */

/* _glmMax: returns the maximum of two floats */
  static GLfloat
_glmMax(GLfloat a, GLfloat b) 
{
  if (a > b)
    return a;
  return b;
}

/* _glmAbs: returns the absolute value of a float */
  static GLfloat
_glmAbs(GLfloat f)
{
  if (f < 0)
    return -f;
  return f;
}

/* _glmDot: compute the dot product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 */
  static GLfloat
_glmDot(GLfloat* u, GLfloat* v)
{
  assert(u);
  assert(v);

  /* compute the dot product */
  return u[X] * v[X] + u[Y] * v[Y] + u[Z] * v[Z];
}

/* _glmCross: compute the cross product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 * n - array of 3 GLfloats (GLfloat n[3]) to return the cross product in
 */
  static GLvoid
_glmCross(GLfloat* u, GLfloat* v, GLfloat* n)
{
  assert(u);
  assert(v);
  assert(n);

  /* compute the cross product (u x v for right-handed [ccw]) */
  n[X] = u[Y] * v[Z] - u[Z] * v[Y];
  n[Y] = u[Z] * v[X] - u[X] * v[Z];
  n[Z] = u[X] * v[Y] - u[Y] * v[X];
}

/* _glmNormalize: normalize a vector
 *
 * n - array of 3 GLfloats (GLfloat n[3]) to be normalized
 */
  static GLvoid
_glmNormalize(GLfloat* n)
{
  GLfloat l;

  assert(n);

  /* normalize */
  l = (GLfloat)sqrt(n[X] * n[X] + n[Y] * n[Y] + n[Z] * n[Z]);
  n[0] /= l;
  n[1] /= l;
  n[2] /= l;
}

/* _glmEqual: compares two vectors and returns GL_TRUE if they are
 * equal (within a certain threshold) or GL_FALSE if not. An epsilon
 * that works fairly well is 0.000001.
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3]) 
 */
  static GLboolean
_glmEqual(GLfloat* u, GLfloat* v, GLfloat epsilon)
{
  if (_glmAbs(u[0] - v[0]) < epsilon &&
      _glmAbs(u[1] - v[1]) < epsilon &&
      _glmAbs(u[2] - v[2]) < epsilon) 
  {
    return GL_TRUE;
  }
  return GL_FALSE;
}

/* _glmWeldVectors: eliminate (weld) vectors that are within an
 * epsilon of each other.
 *
 * vectors    - array of GLfloat[3]'s to be welded
 * numvectors - number of GLfloat[3]'s in vectors
 * epsilon    - maximum difference between vectors 
 *
 */
  GLfloat*
_glmWeldVectors(GLfloat* vectors, GLuint* numvectors, GLfloat epsilon)
{
  GLfloat* copies;
  GLuint   copied;
  GLuint   i, j;

  copies = (GLfloat*)malloc(sizeof(GLfloat) * 3 * (*numvectors + 1));
  memcpy(copies, vectors, (sizeof(GLfloat) * 3 * (*numvectors + 1)));

  copied = 1;
  for (i = 1; i <= *numvectors; i++) {
    for (j = 1; j <= copied; j++) {
      if (_glmEqual(&vectors[3 * i], &copies[3 * j], epsilon)) {
        goto duplicate;
      }
    }

    /* must not be any duplicates -- add to the copies array */
    copies[3 * copied + 0] = vectors[3 * i + 0];
    copies[3 * copied + 1] = vectors[3 * i + 1];
    copies[3 * copied + 2] = vectors[3 * i + 2];
    j = copied;        /* pass this along for below */
    copied++;

duplicate:
    /* set the first component of this vector to point at the correct
       index into the new copies array */
    vectors[3 * i + 0] = (GLfloat)j;
  }

  *numvectors = copied-1;
  return copies;
}

/* _glmFindGroup: Find a group in the model
 */
  GLMgroup*
_glmFindGroup(GLMmodel* model, char* name)
{
  GLMgroup* group;

  assert(model);

  group = model->groups;
  while(group) {
    if (!strcmp(name, group->name))
      break;
    group = group->next;
  }

  return group;
}

/* _glmAddGroup: Add a group to the model
 */
  GLMgroup*
_glmAddGroup(GLMmodel* model, char* name)
{
  GLMgroup* group;

  group = _glmFindGroup(model, name);
  if (!group) {
    group = (GLMgroup*)malloc(sizeof(GLMgroup));
    group->name = stralloc(name);
    group->material = 0;
    group->numtriangles = 0;
    group->triangles = NULL;
    group->next = model->groups;
    model->groups = group;
    model->numgroups++;
  }

  return group;
}

/* _glmFindGroup: Find a material in the model
 */
  GLuint
_glmFindMaterial(GLMmodel* model, char* name)
{
  GLuint i;

  for (i = 0; i < model->nummaterials; i++) {
    if (!strcmp(model->materials[i].name, name))
      goto found;
  }

  /* didn't find the name, so set it as the default material */
  printf("_glmFindMaterial():  can't find material \"%s\".\n", name);
  i = 0;

found:
  return i;
}


/* _glmDirName: return the directory given a path
 *
 * path - filesystem path
 *
 * The return value should be free'd.
 */
  static char*
_glmDirName(char* path)
{
  char* dir;
  char* s;

  dir = stralloc(path);

  s = strrchr(dir, '/');
  if (s)
    s[1] = '\0';
  else
    dir[0] = '\0';

  return dir;
}
/* _glmReadBMP: read a BMP texture image file
 *
 * filename - filename of the BMP texture
 *
 * return 
 *    whether the BMP is read correctly
 */
/*
int _glmReadBMP(GLuint textureArray[], char* filename, int textureID)
{
	// Read the BMP
	int h = 0;
	int w = 0;
	GLubyte* _teximage = ReadBitmapRGB( STLstring2LPCTSTC( string(filename) ), w, h );

	if ( !_teximage )
	{
		printf("Cannot open the BMP texture!!\n");
		return 0;
	}

	static int _teximageWidth=w;                                        
    static int _teximageHeight=h;

	glGenTextures(1, &textureArray[textureID]);
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_2D, textureArray[textureID]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_BGR, 
        GL_UNSIGNED_BYTE, (GLvoid*)_teximage);

    printf("texture %d : %s is loaded\n", textureArray[textureID], filename); 
    
//	delete [] _teximage;
	return 1;
}*/
/* _glmReadPPM: read a PPM texture image file
 *
 * filename - filename of the PPM texture
 *
 * return:
 *    whether the PPM is read correctly
 */
int _glmReadPPM(GLuint textureArray[], char* filename, int textureID)
{
  int h, w, i;
  //int no_read;
  char buf[200], temp[80];
  FILE *fp;
  static int _teximageWidth, _teximageHeight;
  GLubyte *_teximage;

  if ((fp = fopen(filename, "r"))==NULL) {
    printf("File cannot open!!\n");
    return 0;
    /*exit(1);*/
  }
  else {
    fscanf(fp, "%s\n", buf);  /* GET "P6" */
    fscanf(fp, "%s", temp);
    while(temp[0]=='#') {
      while(fgetc(fp)!='\n') ;
      fscanf(fp, "%s", temp);
    }
    w=atoi(temp);                                            
    fscanf(fp, " %d\n", &h); 
    fscanf(fp, "%s\n", temp);

    _teximage=(GLubyte*)malloc(sizeof(GLubyte)*w*h*3);

    for(i=0; i<w*h*3; i++) {                                   
      _teximage[i] = (GLubyte)fgetc(fp);                       
    }                                                          
    _teximageWidth=w;                                        
    _teximageHeight=h;                                       
    fclose(fp);                                              

    /*
    _teximage_new=(GLubyte*)
      malloc(sizeof(GLubyte)*128*128*3);
    gluScaleImage(GL_RGB, w, h, GL_UNSIGNED_BYTE, (GLvoid*)_teximage, 
        128, 128, GL_UNSIGNED_BYTE, (GLvoid*)_teximage_new);          
     */
    
    /* 
     * Generate a texture with the associative texture ID stored in 
     * the array, sets the alignment requirements for the start of each pixel
     * row in memory. Bind the texture to the texture arrays index and 
     * init the texture
     */

    glGenTextures(1, &textureArray[textureID]);
    glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_2D, textureArray[textureID]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); 
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_RGB, 
        GL_UNSIGNED_BYTE, (GLvoid*)_teximage);

    printf("texture %d : %s is loaded\n", textureArray[textureID], filename); 
    /*
    glTexImage2D(GL_TEXTURE_2D, 0, 3, w, h, 0, GL_RGB, 
        GL_UNSIGNED_BYTE, (GLvoid*)_teximage);
    */
    free (_teximage);
  }                                                          
  return 1;
}

/* _glmReadMTL: read a wavefront material library file
 *
 * model - properly initialized GLMmodel structure
 * name  - name of the material library
 */
static void _glmReadMTL(GLMmodel* model, char* name)
{
  FILE* file;
  char* dir;
  char* filename;
  char  buf[128];
  GLuint nummaterials, i;

  dir = _glmDirName(model->pathname);
  filename = (char*)malloc(sizeof(char) * (strlen(dir) + strlen(name) + 1));
  strcpy(filename, dir);
  strcat(filename, name);
  free(dir);

  /* open the file */
  file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "_glmReadMTL() failed: can't open material file \"%s\".\n",
        filename);
    exit(1);
  }
  free(filename);

  /* count the number of materials in the file */
  nummaterials = 1;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case '#':        /* comment */
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
      case 'n':        /* newmtl */
        fgets(buf, sizeof(buf), file);
        nummaterials++;
        sscanf(buf, "%s %s", buf, buf);
        break;
      default:
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
    }
  }

  rewind(file);

  /* allocate memory for the materials */
  model->materials = (GLMmaterial*)malloc(sizeof(GLMmaterial) * nummaterials);
  model->nummaterials = nummaterials;

  /* set the default material */
  for (i = 0; i < nummaterials; i++) {
    model->materials[i].name = NULL;
    model->materials[i].shininess = 0;
    model->materials[i].diffuse[0] = 0.8;
    model->materials[i].diffuse[1] = 0.8;
    model->materials[i].diffuse[2] = 0.8;
    model->materials[i].diffuse[3] = 1.0;
    model->materials[i].ambient[0] = 0.2;
    model->materials[i].ambient[1] = 0.2;
    model->materials[i].ambient[2] = 0.2;
    model->materials[i].ambient[3] = 1.0;
    model->materials[i].specular[0] = 0.0;
    model->materials[i].specular[1] = 0.0;
    model->materials[i].specular[2] = 0.0;
    model->materials[i].specular[3] = 1.0;
    model->materials[i].map_file = NULL;
  }
  model->materials[0].name = stralloc("default");
  /*
   * glGenTextures(nummaterials, textureArray);
   */

  /* now, read in the data */
  nummaterials = 0;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case '#':        /* comment */
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
      case 'n':        /* newmtl */
        fgets(buf, sizeof(buf), file);
        sscanf(buf, "%s %s", buf, buf);
        nummaterials++;
        model->materials[nummaterials].name = stralloc(buf);
        break;
      case 'N':
        fscanf(file, "%f", &model->materials[nummaterials].shininess);
        /* wavefront shininess is from [0, 1000], so scale for OpenGL */
        model->materials[nummaterials].shininess /= 1000.0;
        model->materials[nummaterials].shininess *= 128.0;
        break;
      case 'm':        /* texture map */
        fgets(buf, sizeof(buf), file);
        sscanf(buf, "%s %s", buf, buf);
        model->materials[nummaterials].map_file = stralloc(buf);
		_glmReadPPM(textureArray, buf, nummaterials);
        break;
      case 'K':
        switch(buf[1]) {
          case 'd':
            fscanf(file, "%f %f %f",
                &model->materials[nummaterials].diffuse[0],
                &model->materials[nummaterials].diffuse[1],
                &model->materials[nummaterials].diffuse[2]);
            break;
          case 's':
            fscanf(file, "%f %f %f",
                &model->materials[nummaterials].specular[0],
                &model->materials[nummaterials].specular[1],
                &model->materials[nummaterials].specular[2]);
            break;
          case 'a':
            fscanf(file, "%f %f %f",
                &model->materials[nummaterials].ambient[0],
                &model->materials[nummaterials].ambient[1],
                &model->materials[nummaterials].ambient[2]);
            break;
          default:
            /* eat up rest of line */
            fgets(buf, sizeof(buf), file);
            break;
        }
        break;
      default:
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
    }
  }
}

/* _glmWriteMTL: write a wavefront material library file
 *
 * model      - properly initialized GLMmodel structure
 * modelpath  - pathname of the model being written
 * mtllibname - name of the material library to be written
 */
  static GLvoid
_glmWriteMTL(GLMmodel* model, char* modelpath, char* mtllibname)
{
  FILE* file;
  char* dir;
  char* filename;
  GLMmaterial* material;
  GLuint i;

  dir = _glmDirName(modelpath);
  filename = (char*)malloc(sizeof(char) * (strlen(dir) + strlen(mtllibname)));
  strcpy(filename, dir);
  strcat(filename, mtllibname);
  free(dir);

  /* open the file */
  file = fopen(filename, "w");
  if (!file) {
    fprintf(stderr, "_glmWriteMTL() failed: can't open file \"%s\".\n",
        filename);
    exit(1);
  }
  free(filename);

  /* spit out a header */
  fprintf(file, "#  \n");
  fprintf(file, "#  Wavefront MTL generated by GLM library\n");
  fprintf(file, "#  \n");
  fprintf(file, "#  GLM library copyright (C) 1997 by Nate Robins\n");
  fprintf(file, "#  email: ndr@pobox.com\n");
  fprintf(file, "#  www:   http://www.pobox.com/~ndr\n");
  fprintf(file, "#  \n\n");

  for (i = 0; i < model->nummaterials; i++) {
    material = &model->materials[i];
    fprintf(file, "newmtl %s\n", material->name);
    fprintf(file, "Ka %f %f %f\n", 
        material->ambient[0], material->ambient[1], material->ambient[2]);
    fprintf(file, "Kd %f %f %f\n", 
        material->diffuse[0], material->diffuse[1], material->diffuse[2]);
    fprintf(file, "Ks %f %f %f\n", 
        material->specular[0],material->specular[1],material->specular[2]);
    fprintf(file, "Ns %f\n", material->shininess);
    fprintf(file, "\n");
  }
}


/* _glmFirstPass: first pass at a Wavefront OBJ file that gets all the
 * statistics of the model (such as #vertices, #normals, etc)
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
  static GLvoid
_glmFirstPass(GLMmodel* model, FILE* file) 
{
  GLuint    numvertices;    /* number of vertices in model */
  GLuint    numnormals;      /* number of normals in model */
  GLuint    numtexcoords;    /* number of texcoords in model */
  GLuint    numtriangles;    /* number of triangles in model */
  GLMgroup* group;      /* current group */
  unsigned  v, n, t;
  char      buf[128];

  /* make a default group */  

  group = _glmAddGroup(model, (char*)"default");

  numvertices = numnormals = numtexcoords = numtriangles = 0;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case '#':        /* comment */
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
      case 'v':        /* v, vn, vt */
        switch(buf[1]) {
          case '\0':      /* vertex */
            /* eat up rest of line */
            fgets(buf, sizeof(buf), file);
            numvertices++;
            break;
          case 'n':        /* normal */
            /* eat up rest of line */
            fgets(buf, sizeof(buf), file);
            numnormals++;
            break;
          case 't':        /* texcoord */
            /* eat up rest of line */
            fgets(buf, sizeof(buf), file);
            numtexcoords++;
            break;
          default:
            printf("_glmFirstPass(): Unknown token \"%s\".\n", buf);
            exit(1);
            break;
        }
        break;
      case 'm':
        fgets(buf, sizeof(buf), file);
        sscanf(buf, "%s %s", buf, buf);
        model->mtllibname = stralloc(buf);
        _glmReadMTL(model, buf);
        break;
      case 'u':
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
      case 'g':        /* group */
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        sscanf(buf, "%s", buf);
        group = _glmAddGroup(model, buf);
        break;
      case 'f':        /* face */
        v = n = t = 0;
        fscanf(file, "%s", buf);
        /* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
        if (strstr(buf, "//")) {
          /* v//n */
          sscanf(buf, "%d//%d", &v, &n);
          fscanf(file, "%d//%d", &v, &n);
          fscanf(file, "%d//%d", &v, &n);
          numtriangles++;
          group->numtriangles++;
          while(fscanf(file, "%d//%d", &v, &n) > 0) {
            numtriangles++;
            group->numtriangles++;
          }
        } else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
          /* v/t/n */
          fscanf(file, "%d/%d/%d", &v, &t, &n);
          fscanf(file, "%d/%d/%d", &v, &t, &n);
          numtriangles++;
          group->numtriangles++;
          while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
            numtriangles++;
            group->numtriangles++;
          }
        } else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
          /* v/t */
          fscanf(file, "%d/%d", &v, &t);
          fscanf(file, "%d/%d", &v, &t);
          numtriangles++;
          group->numtriangles++;
          while(fscanf(file, "%d/%d", &v, &t) > 0) {
            numtriangles++;
            group->numtriangles++;
          }
        } else {
          /* v */
          fscanf(file, "%d", &v);
          fscanf(file, "%d", &v);
          numtriangles++;
          group->numtriangles++;
          while(fscanf(file, "%d", &v) > 0) {
            numtriangles++;
            group->numtriangles++;
          }
        }
        break;

      default:
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
    }
  }

#if 0
  /* announce the model statistics */
  printf(" Vertices: %d\n", numvertices);
  printf(" Normals: %d\n", numnormals);
  printf(" Texcoords: %d\n", numtexcoords);
  printf(" Triangles: %d\n", numtriangles);
  printf(" Groups: %d\n", model->numgroups);
#endif

  /* set the stats in the model structure */
  model->numvertices  = numvertices;
  model->numnormals   = numnormals;
  model->numtexcoords = numtexcoords;
  model->numtriangles = numtriangles;

  /* allocate memory for the triangles in each group */
  group = model->groups;
  while(group) {
    group->triangles = (GLuint*)malloc(sizeof(GLuint) * group->numtriangles);
    group->numtriangles = 0;
    group = group->next;
  }
}

/* _glmSecondPass: second pass at a Wavefront OBJ file that gets all
 * the data.
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
  static GLvoid
_glmSecondPass(GLMmodel* model, FILE* file) 
{
  GLuint    numvertices;    /* number of vertices in model */
  GLuint    numnormals;      /* number of normals in model */
  GLuint    numtexcoords;    /* number of texcoords in model */
  GLuint    numtriangles;    /* number of triangles in model */
  GLfloat*  vertices;      /* array of vertices  */
  GLfloat*  normals;      /* array of normals */
  GLfloat*  texcoords;      /* array of texture coordinates */
  GLMgroup* group;      /* current group pointer */
  GLuint    material;      /* current material */
  GLuint    v, n, t;
  char      buf[128];
  GLfloat   tmpYcoord;

  /* set the pointer shortcuts */
  vertices     = model->vertices;
  normals      = model->normals;
  texcoords    = model->texcoords;
  group        = model->groups;
  
  /* on the second pass through the file, read all the data into the
     allocated arrays */
  numvertices = numnormals = numtexcoords = 1;
  numtriangles = 0;
  material = 0;
  while(fscanf(file, "%s", buf) != EOF) {
    switch(buf[0]) {
      case '#':        /* comment */
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
      case 'v':        /* v, vn, vt */
        switch(buf[1]) {
          case '\0':      /* vertex */
            fscanf(file, "%f %f %f", 
                &vertices[3 * numvertices + X], 
                &vertices[3 * numvertices + Y], 
                &vertices[3 * numvertices + Z]);
			   
                numvertices++;
            break;
          case 'n':        /* normal */
            fscanf(file, "%f %f %f", 
                &normals[3 * numnormals + X],
                &normals[3 * numnormals + Y], 
                &normals[3 * numnormals + Z]);
            numnormals++;
            break;
          case 't':        /* texcoord */
            fscanf(file, "%f %f", 
                &texcoords[2 * numtexcoords + X],
                &tmpYcoord);
                texcoords[2 * numtexcoords + Y] = -tmpYcoord;
            numtexcoords++;
            break;
        }
        break;
      case 'u':
        fgets(buf, sizeof(buf), file);
        sscanf(buf, "%s %s", buf, buf);
        group->material = material = _glmFindMaterial(model, buf);
        break;
      case 'g':        /* group */
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        sscanf(buf, "%s", buf);
        group = _glmFindGroup(model, buf);
        group->material = material;
        break;
      case 'f':        /* face */
        v = n = t = 0;
        fscanf(file, "%s", buf);
        /* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
        if (strstr(buf, "//")) {
          /* v//n */
          sscanf(buf, "%d//%d", &v, &n);
          T(numtriangles).vindices[0] = v;
          T(numtriangles).nindices[0] = n;
          fscanf(file, "%d//%d", &v, &n);
          T(numtriangles).vindices[1] = v;
          T(numtriangles).nindices[1] = n;
          fscanf(file, "%d//%d", &v, &n);
          T(numtriangles).vindices[2] = v;
          T(numtriangles).nindices[2] = n;
          group->triangles[group->numtriangles++] = numtriangles;
          numtriangles++;
          while(fscanf(file, "%d//%d", &v, &n) > 0) {
            T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
            T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
            T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
            T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
            T(numtriangles).vindices[2] = v;
            T(numtriangles).nindices[2] = n;
            group->triangles[group->numtriangles++] = numtriangles;
            numtriangles++;
          }
        } else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
          /* v/t/n */
          T(numtriangles).vindices[0] = v;
          T(numtriangles).tindices[0] = t;
          T(numtriangles).nindices[0] = n;
          fscanf(file, "%d/%d/%d", &v, &t, &n);
          T(numtriangles).vindices[1] = v;
          T(numtriangles).tindices[1] = t;
          T(numtriangles).nindices[1] = n;
          fscanf(file, "%d/%d/%d", &v, &t, &n);
          T(numtriangles).vindices[2] = v;
          T(numtriangles).tindices[2] = t;
          T(numtriangles).nindices[2] = n;
          group->triangles[group->numtriangles++] = numtriangles;
          numtriangles++;
          while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
            T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
            T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
            T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
            T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
            T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
            T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
            T(numtriangles).vindices[2] = v;
            T(numtriangles).tindices[2] = t;
            T(numtriangles).nindices[2] = n;
            group->triangles[group->numtriangles++] = numtriangles;
            numtriangles++;
          }
        } else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
          /* v/t */
          T(numtriangles).vindices[0] = v;
          T(numtriangles).tindices[0] = t;
          fscanf(file, "%d/%d", &v, &t);
          T(numtriangles).vindices[1] = v;
          T(numtriangles).tindices[1] = t;
          fscanf(file, "%d/%d", &v, &t);
          T(numtriangles).vindices[2] = v;
          T(numtriangles).tindices[2] = t;
          group->triangles[group->numtriangles++] = numtriangles;
          numtriangles++;
          while(fscanf(file, "%d/%d", &v, &t) > 0) {
            T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
            T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
            T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
            T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
            T(numtriangles).vindices[2] = v;
            T(numtriangles).tindices[2] = t;
            group->triangles[group->numtriangles++] = numtriangles;
            numtriangles++;
          }
        } else {
          /* v */
          sscanf(buf, "%d", &v);
          T(numtriangles).vindices[0] = v;
          fscanf(file, "%d", &v);
          T(numtriangles).vindices[1] = v;
          fscanf(file, "%d", &v);
          T(numtriangles).vindices[2] = v;
          group->triangles[group->numtriangles++] = numtriangles;
          numtriangles++;
          while(fscanf(file, "%d", &v) > 0) {
            T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
            T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
            T(numtriangles).vindices[2] = v;
            group->triangles[group->numtriangles++] = numtriangles;
            numtriangles++;
          }
        }
        break;

      default:
        /* eat up rest of line */
        fgets(buf, sizeof(buf), file);
        break;
    }
  }
  //printf("��%d���㣬x=%f,y=%f,z=%f\n", numvertices, vertices[3 * 1 + 0],
	  //vertices[3 * 1 + Y], vertices[3 * 1 + Z]);
#if 0
  /* announce the memory requirements */
  printf(" Memory: %d bytes\n",
      numvertices  * 3*sizeof(GLfloat) +
      numnormals   * 3*sizeof(GLfloat) * (numnormals ? 1 : 0) +
      numtexcoords * 3*sizeof(GLfloat) * (numtexcoords ? 1 : 0) +
      numtriangles * sizeof(GLMtriangle));
#endif
}

/* public functions */

/* glmUnitize: "unitize" a model by translating it to the origin and
 * scaling it to fit in a unit cube around the origin.  Returns the
 * scalefactor used.
 *
 * model - properly initialized GLMmodel structure 
 */
  GLfloat
glmUnitize(GLMmodel* model, GLfloat center[3])
{
  GLuint  i;
  GLfloat maxx, minx, maxy, miny, maxz, minz;
  GLfloat cx, cy, cz, w, h, d;
  GLfloat scale;
  int i_maxx = 0;
  int i_minx = 0;
  int i_maxy = 0;
  int i_miny = 0;
  int i_maxz = 0;
  int i_minz = 0;
  assert(model);
  assert(model->vertices);

  /* get the max/mins */
  maxx = minx = model->vertices[3 + X];
  maxy = miny = model->vertices[3 + Y];
  maxz = minz = model->vertices[3 + Z];
  for (i = 1; i <= model->numvertices; i++) {
	if (maxx < model->vertices[3 * i + X]){
		maxx = model->vertices[3 * i + X];
		i_maxx = i;
	}
     
	if (minx > model->vertices[3 * i + X]){
		minx = model->vertices[3 * i + X];
		i_minx = i;	
	}      

	if (maxy < model->vertices[3 * i + Y]){
		maxy = model->vertices[3 * i + Y];
		i_maxy = i;
	}
      
	if (miny > model->vertices[3 * i + Y]){
		miny = model->vertices[3 * i + Y];	
		i_miny = i;
	}      

	if (maxz < model->vertices[3 * i + Z]){
		maxz = model->vertices[3 * i + Z];
		i_maxz = i;
	}
      
	if (minz > model->vertices[3 * i + Z]){
		minz = model->vertices[3 * i + Z];	
		i_minz = i;
	}
      
  }

  /* calculate model width, height, and depth */
  //w = _glmAbs(maxx) + _glmAbs(minx);
  //h = _glmAbs(maxy) + _glmAbs(miny);
  //d = _glmAbs(maxz) + _glmAbs(minz);
  w = maxx - minx;
  h = maxy - miny;
  d = maxz - minz;

  /* calculate center of the model */
  cx = (maxx + minx) / 2.0;
  cy = (maxy + miny) / 2.0;
  cz = (maxz + minz) / 2.0;

  /* calculate unitizing scale factor */
  scale = 2.0 / _glmMax(_glmMax(w, h), d);

  /* translate around center then scale */
  for (i = 1; i <= model->numvertices; i++) {
    model->vertices[3 * i + X] -= cx;
    model->vertices[3 * i + Y] -= cy;
    model->vertices[3 * i + Z] -= cz;
    model->vertices[3 * i + X] *= scale;
    model->vertices[3 * i + Y] *= scale;
    model->vertices[3 * i + Z] *= scale;
  }

  center[0] = cx;
  center[1] = cy;
  center[2] = cz;
  maxx -= cx;
  maxx *= scale;
  minx -= cx;
  minx *= scale;
  maxy -= cy;
  maxy *= scale;
  miny -= cy;
  miny *= scale;
  maxz -= cz;
  maxz *= scale;
  minz -= cz;
  minz *= scale;

  printf("maxx = %f,minx = %f\n", maxx, minx);
  printf("maxy = %f,miny = %f\n", maxy, miny);
  printf("maxz = %f,minz = %f\n", maxz, minz);
  
  boxCoordinate.push_back(i_maxx); boxCoordinate.push_back(i_minx);
  boxCoordinate.push_back(i_maxy); boxCoordinate.push_back(i_miny);
  boxCoordinate.push_back(i_maxz); boxCoordinate.push_back(i_minz);
  
  return scale;
}

/* glmDimensions: Calculates the dimensions (width, height, depth) of
 * a model.
 *
 * model      - initialized GLMmodel structure
 * dimensions - array of 3 GLfloats (GLfloat dimensions[3])
 */
  GLvoid
glmDimensions(GLMmodel* model, GLfloat* dimensions)
{
  GLuint i;
  GLfloat maxx, minx, maxy, miny, maxz, minz;

  assert(model);
  assert(model->vertices);
  assert(dimensions);

  /* get the max/mins */
  maxx = minx = model->vertices[3 + X];
  maxy = miny = model->vertices[3 + Y];
  maxz = minz = model->vertices[3 + Z];
  for (i = 1; i <= model->numvertices; i++) {
    if (maxx < model->vertices[3 * i + X])
      maxx = model->vertices[3 * i + X];
    if (minx > model->vertices[3 * i + X])
      minx = model->vertices[3 * i + X];

    if (maxy < model->vertices[3 * i + Y])
      maxy = model->vertices[3 * i + Y];
    if (miny > model->vertices[3 * i + Y])
      miny = model->vertices[3 * i + Y];

    if (maxz < model->vertices[3 * i + Z])
      maxz = model->vertices[3 * i + Z];
    if (minz > model->vertices[3 * i + Z])
      minz = model->vertices[3 * i + Z];
  }

  /* calculate model width, height, and depth */
  dimensions[X] = _glmAbs(maxx) + _glmAbs(minx);
  dimensions[Y] = _glmAbs(maxy) + _glmAbs(miny);
  dimensions[Z] = _glmAbs(maxz) + _glmAbs(minz);
}

/* glmScale: Scales a model by a given amount.
 * 
 * model - properly initialized GLMmodel structure
 * scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
 */
  GLvoid
glmScale(GLMmodel* model, GLfloat scale)
{
  GLuint i;

  for (i = 1; i <= model->numvertices; i++) {
    model->vertices[3 * i + X] *= scale;
    model->vertices[3 * i + Y] *= scale;
    model->vertices[3 * i + Z] *= scale;
  }
}

/* glmReverseWinding: Reverse the polygon winding for all polygons in
 * this model.  Default winding is counter-clockwise.  Also changes
 * the direction of the normals.
 * 
 * model - properly initialized GLMmodel structure 
 */
  GLvoid
glmReverseWinding(GLMmodel* model)
{
  GLuint i, swap;

  assert(model);

  for (i = 0; i < model->numtriangles; i++) {
    swap = T(i).vindices[0];
    T(i).vindices[0] = T(i).vindices[2];
    T(i).vindices[2] = swap;

    if (model->numnormals) {
      swap = T(i).nindices[0];
      T(i).nindices[0] = T(i).nindices[2];
      T(i).nindices[2] = swap;
    }

    if (model->numtexcoords) {
      swap = T(i).tindices[0];
      T(i).tindices[0] = T(i).tindices[2];
      T(i).tindices[2] = swap;
    }
  }

  /* reverse facet normals */
  for (i = 1; i <= model->numfacetnorms; i++) {
    model->facetnorms[3 * i + X] = -model->facetnorms[3 * i + X];
    model->facetnorms[3 * i + Y] = -model->facetnorms[3 * i + Y];
    model->facetnorms[3 * i + Z] = -model->facetnorms[3 * i + Z];
  }

  /* reverse vertex normals */
  for (i = 1; i <= model->numnormals; i++) {
    model->normals[3 * i + X] = -model->normals[3 * i + X];
    model->normals[3 * i + Y] = -model->normals[3 * i + Y];
    model->normals[3 * i + Z] = -model->normals[3 * i + Z];
  }
}

/* glmFacetNormals: Generates facet normals for a model (by taking the
 * cross product of the two vectors derived from the sides of each
 * triangle).  Assumes a counter-clockwise winding.
 *
 * model - initialized GLMmodel structure
 */
  GLvoid
glmFacetNormals(GLMmodel* model)
{
  GLuint  i;
  GLfloat u[3];
  GLfloat v[3];

  assert(model);
  assert(model->vertices);

  /* clobber any old facetnormals */
  if (model->facetnorms)
    free(model->facetnorms);

  /* allocate memory for the new facet normals */
  model->numfacetnorms = model->numtriangles;
  model->facetnorms = (GLfloat*)malloc(sizeof(GLfloat) *
      3 * (model->numfacetnorms + 1));

  for (i = 0; i < model->numtriangles; i++) {
    model->triangles[i].findex = i+1;

    u[X] = model->vertices[3 * T(i).vindices[1] + X] -
      model->vertices[3 * T(i).vindices[0] + X];
    u[Y] = model->vertices[3 * T(i).vindices[1] + Y] -
      model->vertices[3 * T(i).vindices[0] + Y];
    u[Z] = model->vertices[3 * T(i).vindices[1] + Z] -
      model->vertices[3 * T(i).vindices[0] + Z];

    v[X] = model->vertices[3 * T(i).vindices[2] + X] -
      model->vertices[3 * T(i).vindices[0] + X];
    v[Y] = model->vertices[3 * T(i).vindices[2] + Y] -
      model->vertices[3 * T(i).vindices[0] + Y];
    v[Z] = model->vertices[3 * T(i).vindices[2] + Z] -
      model->vertices[3 * T(i).vindices[0] + Z];

    _glmCross(u, v, &model->facetnorms[3 * (i+1)]);
    _glmNormalize(&model->facetnorms[3 * (i+1)]);
  }
}

/* glmVertexNormals: Generates smooth vertex normals for a model.
 * First builds a list of all the triangles each vertex is in.  Then
 * loops through each vertex in the the list averaging all the facet
 * normals of the triangles each vertex is in.  Finally, sets the
 * normal index in the triangle for the vertex to the generated smooth
 * normal.  If the dot product of a facet normal and the facet normal
 * associated with the first triangle in the list of triangles the
 * current vertex is in is greater than the cosine of the angle
 * parameter to the function, that facet normal is not added into the
 * average normal calculation and the corresponding vertex is given
 * the facet normal.  This tends to preserve hard edges.  The angle to
 * use depends on the model, but 90 degrees is usually a good start.
 *
 * model - initialized GLMmodel structure
 * angle - maximum angle (in degrees) to smooth across
 */
  GLvoid
glmVertexNormals(GLMmodel* model, GLfloat angle)
{
  GLMnode*  node;
  GLMnode*  tail;
  GLMnode** members;
  GLfloat*  normals;
  GLuint    numnormals;
  GLfloat   average[3];
  GLfloat   dot, cos_angle;
  GLuint    i, avg;

  assert(model);
  assert(model->facetnorms);

  /* calculate the cosine of the angle (in degrees) */
  cos_angle = cos(angle * M_PI / 180.0);

  /* nuke any previous normals */
  if (model->normals)
    free(model->normals);

  /* allocate space for new normals */
  model->numnormals = model->numtriangles * 3; /* 3 normals per triangle */
  model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));

  /* allocate a structure that will hold a linked list of triangle
     indices for each vertex */
  members = (GLMnode**)malloc(sizeof(GLMnode*) * (model->numvertices + 1));
  for (i = 1; i <= model->numvertices; i++)
    members[i] = NULL;

  /* for every triangle, create a node for each vertex in it */
  for (i = 0; i < model->numtriangles; i++) {
    node = (GLMnode*)malloc(sizeof(GLMnode));
    node->index = i;
    node->next  = members[T(i).vindices[0]];
    members[T(i).vindices[0]] = node;

    node = (GLMnode*)malloc(sizeof(GLMnode));
    node->index = i;
    node->next  = members[T(i).vindices[1]];
    members[T(i).vindices[1]] = node;
	
	node = (GLMnode*)malloc(sizeof(GLMnode));
    node->index = i;
    node->next  = members[T(i).vindices[2]];
    members[T(i).vindices[2]] = node;
  }

  /* calculate the average normal for each vertex */
  numnormals = 1;
  int recordNum = 0;
  for (i = 1; i <= model->numvertices; i++) {
    /* calculate an average normal for this vertex by averaging the
       facet normal of every triangle this vertex is in */	 
    node = members[i];
	if (!node){
		fprintf(stderr, "glmVertexNormals(): vertex w/o a triangle\n");
		model->normals[3 * numnormals + 0] = average[0];
		model->normals[3 * numnormals + 1] = average[1];
		model->normals[3 * numnormals + 2] = average[2];
		numnormals++;
		continue;
	}     
    average[0] = 0.0; average[1] = 0.0; average[2] = 0.0;
    avg = 0;
    while (node) {
      /* only average if the dot product of the angle between the two
         facet normals is greater than the cosine of the threshold
         angle -- or, said another way, the angle between the two
         facet normals is less than (or equal to) the threshold angle */
      dot = _glmDot(&model->facetnorms[3 * T(node->index).findex],
          &model->facetnorms[3 * T(members[i]->index).findex]);
      if (dot > cos_angle) {
        node->averaged = GL_TRUE;
        average[0] += model->facetnorms[3 * T(node->index).findex + 0];
        average[1] += model->facetnorms[3 * T(node->index).findex + 1];
        average[2] += model->facetnorms[3 * T(node->index).findex + 2];
        avg = 1;      /* we averaged at least one normal! */
      } else {
        node->averaged = GL_FALSE;
      }
      node = node->next;
    }

    if (avg) {
      /* normalize the averaged normal */
      _glmNormalize(average);

      /* add the normal to the vertex normals list */
      model->normals[3 * numnormals + 0] = average[0];
      model->normals[3 * numnormals + 1] = average[1];
      model->normals[3 * numnormals + 2] = average[2];
      avg = numnormals;
      numnormals++;
    }

    /* set the normal of this vertex in each triangle it is in */
    node = members[i];
    while (node) {
      if (node->averaged) {
        /* if this node was averaged, use the average normal */
        if (T(node->index).vindices[0] == i)
          T(node->index).nindices[0] = avg;
        else if (T(node->index).vindices[1] == i)
          T(node->index).nindices[1] = avg;
        else if (T(node->index).vindices[2] == i)
          T(node->index).nindices[2] = avg;
      } 
	  else
	  {
        /* if this node wasn't averaged, use the facet normal */
        model->normals[3 * numnormals + 0] = 
          model->facetnorms[3 * T(node->index).findex + 0];
        model->normals[3 * numnormals + 1] = 
          model->facetnorms[3 * T(node->index).findex + 1];
        model->normals[3 * numnormals + 2] = 
          model->facetnorms[3 * T(node->index).findex + 2];
        if (T(node->index).vindices[0] == i)
          T(node->index).nindices[0] = numnormals;
        else if (T(node->index).vindices[1] == i)
          T(node->index).nindices[1] = numnormals;
        else if (T(node->index).vindices[2] == i)
          T(node->index).nindices[2] = numnormals;
        numnormals++;
      }
      node = node->next;
    }
  }

  model->numnormals = numnormals - 1;
  /* free the member information */
  for (i = 1; i <= model->numvertices; i++) {
    node = members[i];
    while (node) {
      tail = node;
      node = node->next;
      free(tail);
    }
  }
  free(members);
  /* pack the normals array (we previously allocated the maximum
     number of normals that could possibly be created (numtriangles *
     3), so get rid of some of them (usually alot unless none of the
     facet normals were averaged)) */
  normals = model->normals;
  model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));
  for (i = 1; i <= model->numnormals; i++) {
    model->normals[3 * i] = normals[3 * i];
    model->normals[3 * i + 1] = normals[3 * i + 1];
    model->normals[3 * i + 2] = normals[3 * i + 2];
  }
  free(normals);
  printf("glmVertexNormals(): %d normals generated\n", model->numnormals);
}


/* glmLinearTexture: Generates texture coordinates according to a
 * linear projection of the texture map.  It generates these by
 * linearly mapping the vertices onto a square.
 *
 * model - pointer to initialized GLMmodel structure
 */
  GLvoid
glmLinearTexture(GLMmodel* model)
{
  GLMgroup *group;
  GLfloat dimensions[3];
  GLfloat x, y, scalefactor;
  GLuint i;

  assert(model);

  if (model->texcoords)
    free(model->texcoords);
  model->numtexcoords = model->numvertices;
  model->texcoords=(GLfloat*)malloc(sizeof(GLfloat)*2*(model->numtexcoords+1));

  glmDimensions(model, dimensions);
  scalefactor = 2.0 / 
    _glmAbs(_glmMax(_glmMax(dimensions[0], dimensions[1]), dimensions[2]));

  /* do the calculations */
  for(i = 1; i <= model->numvertices; i++) {
    x = model->vertices[3 * i + 0] * scalefactor;
    y = model->vertices[3 * i + 2] * scalefactor;
    model->texcoords[2 * i + 0] = (x + 1.0) / 2.0;
    model->texcoords[2 * i + 1] = (y + 1.0) / 2.0;
  }

  /* go through and put texture coordinate indices in all the triangles */
  group = model->groups;
  while(group) {
    for(i = 0; i < group->numtriangles; i++) {
      T(group->triangles[i]).tindices[0] = T(group->triangles[i]).vindices[0];
      T(group->triangles[i]).tindices[1] = T(group->triangles[i]).vindices[1];
      T(group->triangles[i]).tindices[2] = T(group->triangles[i]).vindices[2];
    }    
    group = group->next;
  }

#if 0
  printf("glmLinearTexture(): generated %d linear texture coordinates\n",
      model->numtexcoords);
#endif
}

/* glmSpheremapTexture: Generates texture coordinates according to a
 * spherical projection of the texture map.  Sometimes referred to as
 * spheremap, or reflection map texture coordinates.  It generates
 * these by using the normal to calculate where that vertex would map
 * onto a sphere.  Since it is impossible to map something flat
 * perfectly onto something spherical, there is distortion at the
 * poles.  This particular implementation causes the poles along the X
 * axis to be distorted.
 *
 * model - pointer to initialized GLMmodel structure
 */
  GLvoid
glmSpheremapTexture(GLMmodel* model)
{
  GLMgroup* group;
  GLfloat theta, phi, rho, x, y, z, r;
  GLuint i;

  assert(model);
  assert(model->normals);

  if (model->texcoords)
    free(model->texcoords);
  model->numtexcoords = model->numnormals;
  model->texcoords=(GLfloat*)malloc(sizeof(GLfloat)*2*(model->numtexcoords+1));

  /* do the calculations */
  for (i = 1; i <= model->numnormals; i++) {
    z = model->normals[3 * i + 0];  /* re-arrange for pole distortion */
    y = model->normals[3 * i + 1];
    x = model->normals[3 * i + 2];
    r = sqrt((x * x) + (y * y));
    rho = sqrt((r * r) + (z * z));

    if(r == 0.0) {
      theta = 0.0;
      phi = 0.0;
    } else {
      if(z == 0.0)
        phi = M_PI / 2.0;
      else
        phi = acos(z / rho);

#if WE_DONT_NEED_THIS_CODE
      if(x == 0.0)
        theta = M_PI / 2.0;  /* asin(y / r); */
      else
        theta = acos(x / r);
#endif

      if(y == 0.0)
        theta = M_PI / 2.0;  /* acos(x / r); */
      else
        theta = asin(y / r) + (M_PI / 2.0);
    }

    model->texcoords[2 * i + 0] = theta / M_PI;
    model->texcoords[2 * i + 1] = phi / M_PI;
  }

  /* go through and put texcoord indices in all the triangles */
  group = model->groups;
  while(group) {
    for (i = 0; i < group->numtriangles; i++) {
      T(group->triangles[i]).tindices[0] = T(group->triangles[i]).nindices[0];
      T(group->triangles[i]).tindices[1] = T(group->triangles[i]).nindices[1];
      T(group->triangles[i]).tindices[2] = T(group->triangles[i]).nindices[2];
    }
    group = group->next;
  }

#if 0  
  printf("glmSpheremapTexture(): generated %d spheremap texture coordinates\n",
      model->numtexcoords);
#endif
}

/* glmDelete: Deletes a GLMmodel structure.
 *
 * model - initialized GLMmodel structure
 */
  GLvoid
glmDelete(GLMmodel* model)
{
  GLMgroup* group;
  GLuint i;

  assert(model);

  if (model->pathname)   free(model->pathname);
  if (model->mtllibname) free(model->mtllibname);
  if (model->vertices)   free(model->vertices);
  if (model->normals)    free(model->normals);
  if (model->texcoords)  free(model->texcoords);
  if (model->facetnorms) free(model->facetnorms);
  if (model->triangles)  free(model->triangles);
  if (model->materials) {
    for (i = 0; i < model->nummaterials; i++)
      free(model->materials[i].name);
  }
  free(model->materials);
  while(model->groups) {
    group = model->groups;
    model->groups = model->groups->next;
    free(group->name);
    free(group->triangles);
    free(group);
  }

  free(model);
}

/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
 * Returns a pointer to the created object which should be free'd with
 * glmDelete().
 *
 * filename - name of the file containing the Wavefront .OBJ format data.  
 */
  GLMmodel* 
glmReadOBJ(char* filename)
{ 
  //printf("Hello!glmReadOBJ_start\n");
  GLMmodel* model;
  FILE*     file;

  /* open the file */
  file = fopen(filename, "r");
  if (!file) {
    fprintf(stderr, "glmReadOBJ() failed: can't open data file \"%s\".\n",
        filename);
    exit(1);
  }

#if 0
  /* announce the model name */
  printf("Model: %s\n", filename);
#endif

  /* allocate a new model */
  model = (GLMmodel*)malloc(sizeof(GLMmodel));
  model->pathname      = stralloc(filename);
  model->mtllibname    = NULL;
  model->numvertices   = 0;
  model->vertices      = NULL;
  model->numnormals    = 0;
  model->normals       = NULL;
  model->numtexcoords  = 0;
  model->texcoords     = NULL;
  model->numfacetnorms = 0;
  model->facetnorms    = NULL;
  model->numtriangles  = 0;
  model->triangles     = NULL;
  model->nummaterials  = 0;
  model->materials     = NULL;
  model->numgroups     = 0;
  model->groups        = NULL;
  model->position[0]   = 0.0;
  model->position[1]   = 0.0;
  model->position[2]   = 0.0;

  /* make a first pass through the file to get a count of the number
     of vertices, normals, texcoords & triangles */
  _glmFirstPass(model, file);

  /* allocate memory */
  model->vertices = (GLfloat*)malloc(sizeof(GLfloat) *
      3 * (model->numvertices + 1));
  model->triangles = (GLMtriangle*)malloc(sizeof(GLMtriangle) *
      model->numtriangles);
  if (model->numnormals) {
    model->normals = (GLfloat*)malloc(sizeof(GLfloat) *
        3 * (model->numnormals + 1));
  }
  if (model->numtexcoords) {
    model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) *
        2 * (model->numtexcoords + 1));
  }

  /* rewind to beginning of file and read in the data this pass */
  rewind(file);

  _glmSecondPass(model, file);

  /* close the file */
  fclose(file);
  //printf("Hello!glmReadOBJ_End\n");
  return model;
}

/* glmWriteOBJ: Writes a model description in Wavefront .OBJ format to
 * a file.
 *
 * model    - initialized GLMmodel structure
 * filename - name of the file to write the Wavefront .OBJ format data to
 * mode     - a bitwise or of values describing what is written to the file
 *            GLM_NONE     -  render with only vertices
 *            GLM_FLAT     -  render with facet normals
 *            GLM_SMOOTH   -  render with vertex normals
 *            GLM_TEXTURE  -  render with texture coords
 *            GLM_COLOR    -  render with colors (color material)
 *            GLM_MATERIAL -  render with materials
 *            GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
  GLvoid
glmWriteOBJ(GLMmodel* model, char* filename, GLuint mode)
{
  GLuint    i;
  FILE*     file;
  GLMgroup* group;

  assert(model);

  /* do a bit of warning */
  if (mode & GLM_FLAT && !model->facetnorms) {
    printf("glmWriteOBJ() warning: flat normal output requested "
        "with no facet normals defined.\n");
    mode &= ~GLM_FLAT;
  }
  if (mode & GLM_SMOOTH && !model->normals) {
    printf("glmWriteOBJ() warning: smooth normal output requested "
        "with no normals defined.\n");
    mode &= ~GLM_SMOOTH;
  }
  if (mode & GLM_TEXTURE && !model->texcoords) {
    printf("glmWriteOBJ() warning: texture coordinate output requested "
        "with no texture coordinates defined.\n");
    mode &= ~GLM_TEXTURE;
  }
  if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
    printf("glmWriteOBJ() warning: flat normal output requested "
        "and smooth normal output requested (using smooth).\n");
    mode &= ~GLM_FLAT;
  }

  /* open the file */
  file = fopen(filename, "w");
  if (!file) {
    fprintf(stderr, "glmWriteOBJ() failed: can't open file \"%s\" to write.\n",
        filename);
    exit(1);
  }

  /* spit out a header */
  fprintf(file, "#  \n");
  fprintf(file, "#  Wavefront OBJ generated by GLM library\n");
  fprintf(file, "#  \n");
  fprintf(file, "#  GLM library copyright (C) 1997 by Nate Robins\n");
  fprintf(file, "#  email: ndr@pobox.com\n");
  fprintf(file, "#  www:   http://www.pobox.com/~ndr\n");
  fprintf(file, "#  \n");

  if (mode & GLM_MATERIAL && model->mtllibname) {
    fprintf(file, "\nmtllib %s\n\n", model->mtllibname);
    _glmWriteMTL(model, filename, model->mtllibname);
  }

  /* spit out the vertices */
  fprintf(file, "\n");
  fprintf(file, "# %d vertices\n", model->numvertices);
  for (i = 1; i <= model->numvertices; i++) {
    fprintf(file, "v %f %f %f\n", 
        model->vertices[3 * i + 0],
        model->vertices[3 * i + 1],
        model->vertices[3 * i + 2]);
  }
  /*
  //spit out the smooth/flat normals
  if (mode & GLM_SMOOTH) {
    fprintf(file, "\n");
    fprintf(file, "# %d normals\n", model->numnormals);
    for (i = 1; i <= model->numnormals; i++) {
      fprintf(file, "vn %f %f %f\n", 
          model->normals[3 * i + 0],
          model->normals[3 * i + 1],
          model->normals[3 * i + 2]);
    }
  } 
  else if (mode & GLM_FLAT) {
    fprintf(file, "\n");
    fprintf(file, "# %d normals\n", model->numfacetnorms);
    for (i = 1; i <= model->numnormals; i++) {
      fprintf(file, "vn %f %f %f\n", 
          model->facetnorms[3 * i + 0],
          model->facetnorms[3 * i + 1],
          model->facetnorms[3 * i + 2]);
    }
  }

  spit out the texture coordinates 
  if (mode & GLM_TEXTURE) {
    fprintf(file, "\n");
    fprintf(file, "# %d texcoords\n", model->numtexcoords); // ypchui@20070522: texcoords -> numtexcoords
    for (i = 1; i <= model->numtexcoords; i++) {
      fprintf(file, "vt %f %f\n", 
          model->texcoords[2 * i + 0],
          model->texcoords[2 * i + 1]);
    }
  }

  fprintf(file, "\n");
  fprintf(file, "# %d groups\n", model->numgroups);
  fprintf(file, "# %d faces (triangles)\n", model->numtriangles);
  fprintf(file, "\n");
  */
  group = model->groups;
  while(group) {
    fprintf(file, "g %s\n", group->name);
    if (mode & GLM_MATERIAL)
      fprintf(file, "usemtl %s\n", model->materials[group->material].name);
    for (i = 0; i < group->numtriangles; i++) {
      if (mode & GLM_SMOOTH && mode & GLM_TEXTURE) {
        fprintf(file, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
            T(group->triangles[i]).vindices[0], 
            T(group->triangles[i]).nindices[0], 
            T(group->triangles[i]).tindices[0],
            T(group->triangles[i]).vindices[1],
            T(group->triangles[i]).nindices[1],
            T(group->triangles[i]).tindices[1],
            T(group->triangles[i]).vindices[2],
            T(group->triangles[i]).nindices[2],
            T(group->triangles[i]).tindices[2]);
      } else if (mode & GLM_FLAT && mode & GLM_TEXTURE) {
        fprintf(file, "f %d/%d %d/%d %d/%d\n",
            T(group->triangles[i]).vindices[0],
            T(group->triangles[i]).findex,
            T(group->triangles[i]).vindices[1],
            T(group->triangles[i]).findex,
            T(group->triangles[i]).vindices[2],
            T(group->triangles[i]).findex);
      } else if (mode & GLM_TEXTURE) {
        fprintf(file, "f %d/%d %d/%d %d/%d\n",
            T(group->triangles[i]).vindices[0],
            T(group->triangles[i]).tindices[0],
            T(group->triangles[i]).vindices[1],
            T(group->triangles[i]).tindices[1],
            T(group->triangles[i]).vindices[2],
            T(group->triangles[i]).tindices[2]);
      } else if (mode & GLM_SMOOTH) {
        fprintf(file, "f %d//%d %d//%d %d//%d\n",
            T(group->triangles[i]).vindices[0],
            T(group->triangles[i]).nindices[0],
            T(group->triangles[i]).vindices[1],
            T(group->triangles[i]).nindices[1],
            T(group->triangles[i]).vindices[2], 
            T(group->triangles[i]).nindices[2]);
      } else if (mode & GLM_FLAT) {
        fprintf(file, "f %d//%d %d//%d %d//%d\n",
            T(group->triangles[i]).vindices[0], 
            T(group->triangles[i]).findex,
            T(group->triangles[i]).vindices[1],
            T(group->triangles[i]).findex,
            T(group->triangles[i]).vindices[2],
            T(group->triangles[i]).findex);
      } else {
        fprintf(file, "f %d %d %d\n",
            T(group->triangles[i]).vindices[0],
            T(group->triangles[i]).vindices[1],
            T(group->triangles[i]).vindices[2]);
      }
    }
    fprintf(file, "\n");
    group = group->next;
  }

  fclose(file);
}

/* glmDraw: Renders the model to the current OpenGL context using the
 * mode specified.
 *
 * model    - initialized GLMmodel structure
 * mode     - a bitwise OR of values describing what is to be rendered.
 *            GLM_NONE     -  render with only vertices
 *            GLM_FLAT     -  render with facet normals
 *            GLM_SMOOTH   -  render with vertex normals
 *            GLM_TEXTURE  -  render with texture coords
 *            GLM_COLOR    -  render with colors (color material)
 *            GLM_MATERIAL -  render with materials
 *            GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
  GLvoid
glmDraw(GLMmodel* model, GLuint mode)
{
  GLuint i;
  GLMgroup* group;

  assert(model);
  assert(model->vertices);

  /* do a bit of warning */
  if (mode & GLM_FLAT && !model->facetnorms) {
    printf("glmDraw() warning: flat render mode requested "
        "with no facet normals defined.\n");
    mode &= ~GLM_FLAT;
  }
  if (mode & GLM_SMOOTH && !model->normals) {
    printf("glmDraw() warning: smooth render mode requested "
        "with no normals defined.\n");
    mode &= ~GLM_SMOOTH;
  }
  if (mode & GLM_TEXTURE && !model->texcoords) {
    printf("glmDraw() warning: texture render mode requested "
        "with no texture coordinates defined.\n");
    mode &= ~GLM_TEXTURE;
  }
  if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
    printf("glmDraw() warning: flat render mode requested "
        "and smooth render mode requested (using smooth).\n");
    mode &= ~GLM_FLAT;
  }
  if (mode & GLM_COLOR && !model->materials) {
    printf("glmDraw() warning: color render mode requested "
        "with no materials defined.\n");
    mode &= ~GLM_COLOR;
  }
  if (mode & GLM_MATERIAL && !model->materials) {
    printf("glmDraw() warning: material render mode requested "
        "with no materials defined.\n");
    mode &= ~GLM_MATERIAL;
  }
  if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
    printf("glmDraw() warning: color and material render mode requested "
        "using only material mode\n");
    mode &= ~GLM_COLOR;
  }
  if (mode & GLM_COLOR)
    glEnable(GL_COLOR_MATERIAL);
  if (mode & GLM_MATERIAL)
    glDisable(GL_COLOR_MATERIAL);
  
  glPushMatrix();
  glTranslatef(model->position[0], model->position[1], model->position[2]);

  group = model->groups;
  while (group) {
    if (mode & GLM_MATERIAL) {
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, 
          model->materials[group->material].ambient);
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, 
          model->materials[group->material].diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, 
          model->materials[group->material].specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 
          model->materials[group->material].shininess);
    }

    if (mode & GLM_COLOR) {
      glColor3fv(model->materials[group->material].diffuse);
    }

    /* add code here to bind texture for each model */
    if (mode & GLM_TEXTURE) {
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, textureArray[group->material]);
      /*
      printf("texture %d : %s is bound\n", textureArray[group->material], 
          model->materials[group->material].map_file); 
      */
    }
    glBegin(GL_TRIANGLES);
    for (i = 0; i < group->numtriangles; i++) {
      if (mode & GLM_FLAT)
        glNormal3fv(&model->facetnorms[3 * T(group->triangles[i]).findex]);

      if (mode & GLM_SMOOTH)
        glNormal3fv(&model->normals[3 * T(group->triangles[i]).nindices[0]]);
      if (mode & GLM_TEXTURE)
        glTexCoord2fv(&model->texcoords[2*T(group->triangles[i]).tindices[0]]);
      glVertex3fv(&model->vertices[3 * T(group->triangles[i]).vindices[0]]);
#if 0
      printf("%f %f %f\n", 
          model->vertices[3 * T(group->triangles[i]).vindices[0] + X],
          model->vertices[3 * T(group->triangles[i]).vindices[0] + Y],
          model->vertices[3 * T(group->triangles[i]).vindices[0] + Z]);
#endif

      if (mode & GLM_SMOOTH)
        glNormal3fv(&model->normals[3 * T(group->triangles[i]).nindices[1]]);
      if (mode & GLM_TEXTURE)
        glTexCoord2fv(&model->texcoords[2*T(group->triangles[i]).tindices[1]]);
      glVertex3fv(&model->vertices[3 * T(group->triangles[i]).vindices[1]]);
#if 0
      printf("%f %f %f\n", 
          model->vertices[3 * T(group->triangles[i]).vindices[1] + X],
          model->vertices[3 * T(group->triangles[i]).vindices[1] + Y],
          model->vertices[3 * T(group->triangles[i]).vindices[1] + Z]);
#endif

      if (mode & GLM_SMOOTH)
        glNormal3fv(&model->normals[3 * T(group->triangles[i]).nindices[2]]);
      if (mode & GLM_TEXTURE)
        glTexCoord2fv(&model->texcoords[2*T(group->triangles[i]).tindices[2]]);
        glVertex3fv(&model->vertices[3 * T(group->triangles[i]).vindices[2]]);
#if 0
      printf("%f %f %f\n", 
          model->vertices[3 * T(group->triangles[i]).vindices[2] + X],
          model->vertices[3 * T(group->triangles[i]).vindices[2] + Y],
          model->vertices[3 * T(group->triangles[i]).vindices[2] + Z]);
#endif

    }
    glEnd();
    group = group->next;
  }
  
  glPopMatrix();
}

/* glmList: Generates and returns a display list for the model using
 * the mode specified.
 *
 * model    - initialized GLMmodel structure
 * mode     - a bitwise OR of values describing what is to be rendered.
 *            GLM_NONE     -  render with only vertices
 *            GLM_FLAT     -  render with facet normals
 *            GLM_SMOOTH   -  render with vertex normals
 *            GLM_TEXTURE  -  render with texture coords
 *            GLM_COLOR    -  render with colors (color material)
 *            GLM_MATERIAL -  render with materials
 *            GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *            GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
  GLuint
glmList(GLMmodel* model, GLuint mode)
{
  GLuint list;

  list = glGenLists(1);
  glNewList(list, GL_COMPILE);
  glmDraw(model, mode);
  glEndList();

  return list;
}

/* glmWeld: eliminate (weld) vectors that are within an epsilon of
 * each other.
 *
 * model      - initialized GLMmodel structure
 * epsilon    - maximum difference between vertices
 *              ( 0.00001 is a good start for a unitized model)
 *
 */
  GLvoid
glmWeld(GLMmodel* model, GLfloat epsilon)
{
  GLfloat* vectors;
  GLfloat* copies;
  GLuint   numvectors;
  GLuint   i;

  /* vertices */
  numvectors = model->numvertices;
  vectors    = model->vertices;
  copies = _glmWeldVectors(vectors, &numvectors, epsilon);

  printf("glmWeld(): %d redundant vertices.\n", 
      model->numvertices - numvectors - 1);

  for (i = 0; i < model->numtriangles; i++) {
    T(i).vindices[0] = (GLuint)vectors[3 * T(i).vindices[0] + 0];
    T(i).vindices[1] = (GLuint)vectors[3 * T(i).vindices[1] + 0];
    T(i).vindices[2] = (GLuint)vectors[3 * T(i).vindices[2] + 0];
  }

  /* free space for old vertices */
  free(vectors);

  /* allocate space for the new vertices */
  model->numvertices = numvectors;
  model->vertices = (GLfloat*)malloc(sizeof(GLfloat) * 
      3 * (model->numvertices + 1));

  /* copy the optimized vertices into the actual vertex list */
  for (i = 1; i <= model->numvertices; i++) {
    model->vertices[3 * i + 0] = copies[3 * i + 0];
    model->vertices[3 * i + 1] = copies[3 * i + 1];
    model->vertices[3 * i + 2] = copies[3 * i + 2];
  }

  free(copies);
}


#if 0
/* normals */
if (model->numnormals) {
  numvectors = model->numnormals;
  vectors    = model->normals;
  copies = _glmOptimizeVectors(vectors, &numvectors);

  printf("glmOptimize(): %d redundant normals.\n", 
      model->numnormals - numvectors);

  for (i = 0; i < model->numtriangles; i++) {
    T(i).nindices[0] = (GLuint)vectors[3 * T(i).nindices[0] + 0];
    T(i).nindices[1] = (GLuint)vectors[3 * T(i).nindices[1] + 0];
    T(i).nindices[2] = (GLuint)vectors[3 * T(i).nindices[2] + 0];
  }

  /* free space for old normals */
  free(vectors);

  /* allocate space for the new normals */
  model->numnormals = numvectors;
  model->normals = (GLfloat*)malloc(sizeof(GLfloat) * 
      3 * (model->numnormals + 1));

  /* copy the optimized vertices into the actual vertex list */
  for (i = 1; i <= model->numnormals; i++) {
    model->normals[3 * i + 0] = copies[3 * i + 0];
    model->normals[3 * i + 1] = copies[3 * i + 1];
    model->normals[3 * i + 2] = copies[3 * i + 2];
  }

  free(copies);
}

/* texcoords */
if (model->numtexcoords) {
  numvectors = model->numtexcoords;
  vectors    = model->texcoords;
  copies = _glmOptimizeVectors(vectors, &numvectors);

  printf("glmOptimize(): %d redundant texcoords.\n", 
      model->numtexcoords - numvectors);

  for (i = 0; i < model->numtriangles; i++) {
    for (j = 0; j < 3; j++) {
      T(i).tindices[j] = (GLuint)vectors[3 * T(i).tindices[j] + 0];
    }
  }

  /* free space for old texcoords */
  free(vectors);

  /* allocate space for the new texcoords */
  model->numtexcoords = numvectors;
  model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) * 
      2 * (model->numtexcoords + 1));

  /* copy the optimized vertices into the actual vertex list */
  for (i = 1; i <= model->numtexcoords; i++) {
    model->texcoords[2 * i + 0] = copies[2 * i + 0];
    model->texcoords[2 * i + 1] = copies[2 * i + 1];
  }

  free(copies);
}
#endif

#if 0
/* look for unused vertices */
/* look for unused normals */
/* look for unused texcoords */
for (i = 1; i <= model->numvertices; i++) {
  for (j = 0; j < model->numtriangles; i++) {
    if (T(j).vindices[0] == i || 
        T(j).vindices[1] == i || 
        T(j).vindices[1] == i)
      break;
  }
}
#endif

vector<int> glmBox(){
	return boxCoordinate;
}

//�����жϱǼ���box���򣬼�ȡֵ��Χ��
bool glmNosePoint_Box(GLMmodel* model,double x, double y, double z){
	int iMax_x = boxCoordinate.at(0);
	int iMin_x = boxCoordinate.at(1);
	double Max_x = model->vertices[3 * iMax_x + X];
	double Min_x = model->vertices[3 * iMin_x + X];
	double xMax = Max_x - (Max_x - Min_x) / 3;
	double xMin = Min_x + (Max_x - Min_x) / 3;

	int iMax_y = boxCoordinate.at(2);
	int iMin_y = boxCoordinate.at(3);
	double Max_y = model->vertices[3 * iMax_y + Y];
	double Min_y = model->vertices[3 * iMin_y + Y];
	double yMax = Max_y - (Max_y - Min_y) / 2;
	double yMin = Min_y + (Max_y - Min_y) / 3;

	int iMax_z = boxCoordinate.at(4);
	int iMin_z = boxCoordinate.at(5);
	double Max_z = model->vertices[3 * iMax_z + Z];
	double Min_z = model->vertices[3 * iMin_z + Z];
	double zJu = Max_z - (Max_z - Min_z) / 2;
	if (y<= yMax&&y >= yMin
		&&x <= xMax&&x >= xMin
		&&z >= zJu){
		return true;
	}
	else{
		return false;
	}

}

int glmNosePoint(GLMmodel* model){
	int nosePointInec = 0;
	GLfloat maxx, minx, maxy, miny, maxz, minz;
	int i_maxx = 1;
	int i_minx = 1;
	int i_maxy = 1;
	int i_miny = 1;
	int i_maxz = 1;
	int i_minz = 1;	

	maxx = model->vertices[3 * boxCoordinate.at(1)];
	minx = model->vertices[3 * boxCoordinate.at(0)];
	maxy = model->vertices[3 * boxCoordinate.at(3)+1];
	miny = model->vertices[3 * boxCoordinate.at(2)+1];
	maxz = model->vertices[3 * boxCoordinate.at(5)+2];
	minz = model->vertices[3 * boxCoordinate.at(4)+2];	

	//printf("maxz = %f, minz = %f\n", maxz, minz);


	for (int i = 1; i <= model->numvertices; i++) {
		if (glmNosePoint_Box(model, model->vertices[3 * i + X], model->vertices[3 * i + Y], 
			model->vertices[3 * i + Z])){
			if (maxx < model->vertices[3 * i + X]){
				maxx = model->vertices[3 * i + X];
				i_maxx = i;
			}
			if (minx > model->vertices[3 * i + X]){
				minx = model->vertices[3 * i + X];
				i_minx = i;
			}
			if (maxy < model->vertices[3 * i + Y]){
				maxy = model->vertices[3 * i + Y];
				i_maxy = i;
			}
			if (miny > model->vertices[3 * i + Y]){
				miny = model->vertices[3 * i + Y];
				i_miny = i;
			}
			if (maxz < model->vertices[3 * i + Z]){
				maxz = model->vertices[3 * i + Z];
				i_maxz = i;
			}
			if (minz > model->vertices[3 * i + Z]){
				minz = model->vertices[3 * i + Z];
				i_minz = i;
			}		
		}
	}
	//�ֱ����������С��x��y��zֵ�ĵ�
	//return i_maxx;
	//return i_minx;
	//return i_maxy;
	//return i_miny;
	return i_maxz;
	//return i_minz;
}

vector<vector <int>> glmPointNeibor(GLMmodel* model){
	vector<vector <int>> temp;
	for (int i = 0; i <= model->numvertices; i++){
		vector<int> t;
		temp.push_back(t);	
	}

	for (int i = 0; i < model->numtriangles; i++){
		int b1 = model->triangles[i].vindices[0];
		int b2 = model->triangles[i].vindices[1];
		int b3 = model->triangles[i].vindices[2];		
		temp[b1].push_back(b2);
		temp[b1].push_back(b3);
		temp[b2].push_back(b1);
		temp[b2].push_back(b3);
		temp[b3].push_back(b1);
		temp[b3].push_back(b2);
	}

	//printf("glmPointNeibor:%d���ھ�Ϊ:\n",3300);
	//for (int i = 0; i < temp.at(3300).size(); i++){
		//printf("%d\n", temp.at(3300).at(i));	
	//}
	return temp;
}

int glmIndexPoint(GLMmodel* model, double x, double y, double z){
	int indexPoint = 1;
	double xt = model->vertices[3 * 1];
	double yt = model->vertices[3 * 1 + 1];
	double zt = model->vertices[3 * 1 + 2];
	double minLength = (x - xt)* (x - xt) + (y - yt)* (y - yt) + (z - zt)* (z - zt);
	for (int i = 1; i <= model->numvertices; i++){
		xt = model->vertices[3 * i];
		yt = model->vertices[3 * i + 1];
		zt = model->vertices[3 * i + 2];
		double length = (x - xt)* (x - xt) + (y - yt)* (y - yt) + (z - zt)* (z - zt);
		if (length < minLength){
			indexPoint = i;
			minLength = length;
		}
	}
	return indexPoint;
}

void glmTurnNorm2Trangular(GLMmodel* model) {
	for (int i = 0; i < model->numtriangles; i++) {
		int b1 = model->triangles[i].vindices[0];
		int b2 = model->triangles[i].vindices[1];
		int b3 = model->triangles[i].vindices[2];
		model->triangles[i].vindices[0] = b3;
		model->triangles[i].vindices[1] = b2;
		model->triangles[i].vindices[2] = b1;
	}
}