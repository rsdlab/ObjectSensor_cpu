
//
// Macros for vectors and matrices
//
// Jaeil Choi
// last modified in July, 2006
//
// After using classes (Vector3D and Matrix3D, for example) for a while,
// I gave up those classes (at least for basic vector/matrix operations),
// and returned to simple macros. Macros are faster and easy to use.
// They may slightly increase the size after compilation, but who really
// cares the size of the program these days?
//

#ifndef VECTOR_MATRIX_MACROS_H
#define VECTOR_MATRIX_MACROS_H
//
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

/* ================================================================ */
/*  2D                                                              */
/* ================================================================ */

#define G2V_SET(r,x,y)          do { (r)[0] = (x);  (r)[1] = (y); } while(0)
#define G2V_COPY(d,s)           do { (d)[0] = (s)[0];  (d)[1] = (s)[1]; } while(0)

#define G2V_ADD(r,a,b)          do { (r)[0] = (a)[0] + (b)[0];  (r)[1] = (a)[1] + (b)[1]; } while(0)
#define G2V_SUB(r,a,b)          do { (r)[0] = (a)[0] - (b)[0];  (r)[1] = (a)[1] - (b)[1]; } while(0)
#define G2V_DIV_VALUE(a,v)      do { (a)[0] /= (v);  (a)[1] /= (v); } while(0)
#define G2V_SCALED_ADD(r,a,s,b) do { (r)[0] = (a)[0] + (b)[0]*(s);  (r)[1] = (a)[1] + (b)[1]*(s); } while(0)

#define G2V_DISTANCE(a,b)       (sqrt(((b)[0]-(a)[0])*((b)[0]-(a)[0]) + ((b)[1]-(a)[1])*((b)[1]-(a)[1])))
#define G2V_LENGTH(a)           (sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1]))
#define G2V_NORMALIZED(a)       do { double len = G2V_LENGTH(a);  if (len > 0) G2V_DIV_VALUE( a, len ); } while(0)
#define G2V_AVERAGE4(r,a,b,c,d) do { (r)[0] = ((a)[0] + (b)[0] + (c)[0] + d[0])/4;  (r)[1] = ((a)[1] + (b)[1] + (c)[1] + d[1])/4; } while(0)
#define G2V_ROTATION_ANGLE(v) \
  (((v)[0] >= 0) ? asin((v)[1]) : \
   (((v)[1] >= 0) ? M_PI - asin((v)[1]) : -M_PI - asin((v)[1])))	// -PI ~ +PI

#define G2M_SET(M, a00, a01,  a10, a11) \
                       do { (M)[0] = a00;  (M)[1] = a01; \
                            (M)[2] = a10;  (M)[3] = a11; } while(0)
#define G2M_COPY(R,A)  \
                       do { (R)[0] = (A)[0];  (R)[1] = (A)[1]; \
                            (R)[2] = (A)[2];  (R)[3] = (A)[3]; } while(0)
#define G2M_MUL_VALUE(A,v)  \
                       do { (A)[0] *= (v);  (A)[1] *= (v); \
                            (A)[2] *= (v);  (A)[3] *= (v); } while(0)
#define G2M_MUL_MV(r, M, a) \
                       do { (r)[0] = (M)[0] * (a)[0] + (M)[1] * (a)[1]; \
                            (r)[1] = (M)[2] * (a)[0] + (M)[3] * (a)[1]; } while(0)
#define G2M_MUL_MM(R, A, B) \
                       do { (R)[0] = (A)[0] * (B)[0] + (A)[1] * (B)[2]; \
                            (R)[1] = (A)[0] * (B)[1] + (A)[1] * (B)[3]; \
                            (R)[2] = (A)[2] * (B)[0] + (A)[3] * (B)[2]; \
                            (R)[3] = (A)[2] * (B)[1] + (A)[3] * (B)[3]; } while(0)
#define G2M_ROTATION_ANGLE(R) \
  (((R)[0] >= 0) ? asin((R)[2]) : \
   (((R)[2] >= 0) ? M_PI - asin((R)[2]) : -M_PI - asin((R)[2])))	// -PI ~ +PI


/* ================================================================ */
/*  3D                                                              */
/* ================================================================ */

#define G3V_SET(r,x,y,z)        do { (r)[0] = (x);  (r)[1] = (y);  (r)[2] = (z); } while(0)
#define G3V_COPY(d,s)           do { (d)[0] = (s)[0];  (d)[1] = (s)[1];  (d)[2] = (s)[2]; } while(0)

#define G3V_ADD(r,a,b)          do { (r)[0] = (a)[0] + (b)[0];  (r)[1] = (a)[1] + (b)[1];  (r)[2] = (a)[2] + (b)[2]; } while(0)
#define G3V_SUB(r,a,b)          do { (r)[0] = (a)[0] - (b)[0];  (r)[1] = (a)[1] - (b)[1];  (r)[2] = (a)[2] - (b)[2]; } while(0)
#define G3V_MUL_VALUE(a,v)      do { (a)[0] *= (v);  (a)[1] *= (v);  (a)[2] *= (v); } while(0)
#define G3V_DIV_VALUE(a,v)      do { (a)[0] /= (v);  (a)[1] /= (v);  (a)[2] /= (v); } while(0)
#define G3V_SCALED_ADD(r,a,s,b) do { (r)[0] = (a)[0] + (b)[0]*(s);  (r)[1] = (a)[1] + (b)[1]*(s);  (r)[2] = (a)[2] + (b)[2]*(s); } while(0)
//
#define G3V_DOT(a,b)            ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2])
#define G3V_CROSS(r,a,b)        do { (r)[0] = (a)[1]*(b)[2]-(a)[2]*(b)[1]; (r)[1] = (a)[2]*(b)[0]-(a)[0]*(b)[2]; (r)[2] = (a)[0]*(b)[1]-(a)[1]*(b)[0]; } while(0)
#define G3V_PLANE_POINT(p,a)    (p[0]*(a)[0] + p[1]*(a)[1] + p[2]*(a)[2] + p[3])
//
#define G3V_DISTANCE(a,b)       (sqrt(((b)[0]-(a)[0])*((b)[0]-(a)[0]) + ((b)[1]-(a)[1])*((b)[1]-(a)[1]) + ((b)[2]-(a)[2])*((b)[2]-(a)[2])))
#define G3V_LENGTH(a)           (sqrt((a)[0]*(a)[0] + (a)[1]*(a)[1] + (a)[2]*(a)[2]))
#define G3V_NORMALIZE(a)        do { float  len = (float)G3V_LENGTH(a);  if (len > 0) G3V_DIV_VALUE( a, len ); } while(0)
#define G3V_NORMALIZE_WITH_TYPE(a,T_t)  do { T_t len = (T_t)G3V_LENGTH(a);  if (len > 0) G3V_DIV_VALUE( a, len ); } while(0)
//
#define G3V_AVERAGE2(r,a,b)     do { (r)[0] = ((a)[0] + (b)[0])/2;  (r)[1] = ((a)[1] + (b)[1])/2;  (r)[2] = ((a)[2] + (b)[2])/2; } while(0)
//
#define G3M_SET(m, a00,a01,a02,  a10,a11,a12,  a20,a21,a22) \
                       do { (m)[0] = a00;  (m)[1] = a01;  (m)[2] = a02; \
                            (m)[3] = a10;  (m)[4] = a11;  (m)[5] = a12; \
                            (m)[6] = a20;  (m)[7] = a21;  (m)[8] = a22; } while(0)
#define G3M_SET_ID(m)  do { (m)[0] = 1;  (m)[1] = 0;  (m)[2] = 0; \
                            (m)[3] = 0;  (m)[4] = 1;  (m)[5] = 0; \
                            (m)[6] = 0;  (m)[7] = 0;  (m)[8] = 1; } while(0)
#define G3M_COPY(R, A) \
                       do { (R)[0] = (A)[0];  (R)[1] = (A)[1];  (R)[2] = (A)[2]; \
                            (R)[3] = (A)[3];  (R)[4] = (A)[4];  (R)[5] = (A)[5]; \
                            (R)[6] = (A)[6];  (R)[7] = (A)[7];  (R)[8] = (A)[8]; } while(0)
#define G3M_MUL_MV(r, M, a) \
                       do { (r)[0] = (M)[0] * (a)[0] + (M)[1] * (a)[1] + (M)[2] * (a)[2]; \
                            (r)[1] = (M)[3] * (a)[0] + (M)[4] * (a)[1] + (M)[5] * (a)[2]; \
                            (r)[2] = (M)[6] * (a)[0] + (M)[7] * (a)[1] + (M)[8] * (a)[2]; } while(0)
#define G3M_MUL_MM(R, A, B) \
                       do { (R)[0] = (A)[0] * (B)[0] + (A)[1] * (B)[3] + (A)[2] * (B)[6]; \
                            (R)[1] = (A)[0] * (B)[1] + (A)[1] * (B)[4] + (A)[2] * (B)[7]; \
                            (R)[2] = (A)[0] * (B)[2] + (A)[1] * (B)[5] + (A)[2] * (B)[8]; \
                            (R)[3] = (A)[3] * (B)[0] + (A)[4] * (B)[3] + (A)[5] * (B)[6]; \
                            (R)[4] = (A)[3] * (B)[1] + (A)[4] * (B)[4] + (A)[5] * (B)[7]; \
                            (R)[5] = (A)[3] * (B)[2] + (A)[4] * (B)[5] + (A)[5] * (B)[8]; \
                            (R)[6] = (A)[6] * (B)[0] + (A)[7] * (B)[3] + (A)[8] * (B)[6]; \
                            (R)[7] = (A)[6] * (B)[1] + (A)[7] * (B)[4] + (A)[8] * (B)[7]; \
                            (R)[8] = (A)[6] * (B)[2] + (A)[7] * (B)[5] + (A)[8] * (B)[8]; } while(0)
#define G3M_MUL_MMt(R, A, B) \
                       do { (R)[0] = (A)[0] * (B)[0] + (A)[1] * (B)[1] + (A)[2] * (B)[2]; \
                            (R)[1] = (A)[0] * (B)[3] + (A)[1] * (B)[4] + (A)[2] * (B)[5]; \
                            (R)[2] = (A)[0] * (B)[6] + (A)[1] * (B)[7] + (A)[2] * (B)[8]; \
                            (R)[3] = (A)[3] * (B)[0] + (A)[4] * (B)[1] + (A)[5] * (B)[2]; \
                            (R)[4] = (A)[3] * (B)[3] + (A)[4] * (B)[4] + (A)[5] * (B)[5]; \
                            (R)[5] = (A)[3] * (B)[6] + (A)[4] * (B)[7] + (A)[5] * (B)[8]; \
                            (R)[6] = (A)[6] * (B)[0] + (A)[7] * (B)[1] + (A)[8] * (B)[2]; \
                            (R)[7] = (A)[6] * (B)[3] + (A)[7] * (B)[4] + (A)[8] * (B)[5]; \
                            (R)[8] = (A)[6] * (B)[6] + (A)[7] * (B)[7] + (A)[8] * (B)[8]; } while(0)
#define G3M_XFORM_MVV(c, R, a, b) \
                      do { G3M_MUL_MV(c, R, a); G3V_ADD( c, c, b ); } while(0)
#define G3M_XFORM_MERGE(Rac, Tac,   Rab, Tab,   Rbc, Tbc) \
                      do { G3M_MUL_MM(Rac, Rab, Rbc); G3M_XFORM_MVV(Tac, Rab, Tbc, Tab); } while(0)
#define G3M_XFORM_PRINTF(R, T) \
                      do { printf("  [ %5.2f %5.2f %5.2f  %6.2f ] \n",(R)[0], (R)[1], (R)[2], (T)[0]); printf("  [ %5.2f %5.2f %5.2f  %6.2f ] \n",(R)[3], (R)[4], (R)[5], (T)[1]); printf("  [ %5.2f %5.2f %5.2f  %6.2f ] \n",(R)[6], (R)[7], (R)[8], (T)[2]);  } while(0)

/* ================================================================ */
/*  4D                                                              */
/* ================================================================ */

#define G4V_SET(r,x,y,z,w)      do { (r)[0] = (x);  (r)[1] = (y);  (r)[2] = (z);  (r)[3] = (w); } while(0)
#define G4V_COPY(d,s)           do { (d)[0] = (s)[0];  (d)[1] = (s)[1];  (d)[2] = (s)[2];  (d)[3] = (s)[3]; } while(0)

/* ================================================================ */
/*                                                                  */
/* ================================================================ */
#define G5V_SET(p,v0,v1,v2,v3,v4)    do { (p)[0] = v0; (p)[1] = v1; (p)[2] = v2; (p)[3] = v3; (p)[4] = v4; } while(0)
#define G6V_SET(p,v0,v1,v2,v3,v4,v5) do { (p)[0] = v0; (p)[1] = v1; (p)[2] = v2; (p)[3] = v3; (p)[4] = v4; (p)[5] = v5; } while(0)
#define G8V_SET(p,v0,v1,v2,v3,v4,v5,v6,v7) do { (p)[0] = v0; (p)[1] = v1; (p)[2] = v2; (p)[3] = v3; (p)[4] = v4; (p)[5] = v5; (p)[6] = v6; (p)[7] = v7; } while(0)

#endif  // VECTOR_MATRIX_MACROS_H
