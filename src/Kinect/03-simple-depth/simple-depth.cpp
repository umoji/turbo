#include <math.h>
#include <XnOS.h>
#include <XnCppWrapper.h>

#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
    #include <GLUT/glut.h>
#else
    #include <GL/glut.h>
#endif

using namespace xn;


#define SAMPLE_XML_PATH "../../config/SamplesConfig.xml"

#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480


float* g_pDepthHist;
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;
XnDepthPixel g_nZRes;

Context g_context;
ScriptNode g_scriptNode;
DepthGenerator g_depth;
DepthMetaData g_depthMD;


void idle (void)
{
    glutPostRedisplay();
}

void display (void)
{
    XnStatus rc = XN_STATUS_OK;

    // Read a new frame
    rc = g_context.WaitAnyUpdateAll();
    if (rc != XN_STATUS_OK) {
        printf("Read failed: %s\n", xnGetStatusString(rc));
        return;
    }

    g_depth.GetMetaData(g_depthMD);

    const XnDepthPixel* pDepth = g_depthMD.Data();

    // Copied from SimpleViewer
    // Clear the OpenGL buffers
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Setup the OpenGL viewpoint
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

    // Calculate the accumulative histogram (the yellow display...)
    // デプスのヒストグラムを計算し，デプス画像を生成する
    xnOSMemSet(g_pDepthHist, 0, g_nZRes * sizeof(float));

    unsigned int nNumberOfPoints = 0;
    for (XnUInt y = 0; y < g_depthMD.YRes(); ++y) {
        for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth) {
            if (*pDepth != 0) {
                g_pDepthHist[*pDepth]++;
                nNumberOfPoints++;
            }
        }
    }
    for (int nIndex=1; nIndex<g_nZRes; ++nIndex) {
        g_pDepthHist[nIndex] += g_pDepthHist[nIndex - 1];
    }
    if (nNumberOfPoints) {
        for (int nIndex=1; nIndex<g_nZRes; ++nIndex) {
            g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
        }
    }

    xnOSMemSet(g_pTexMap, 0, g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

    const XnDepthPixel* pDepthRow = g_depthMD.Data();
    XnRGB24Pixel* pTexRow = g_pTexMap + g_depthMD.YOffset() * g_nTexMapX;

    for (XnUInt y = 0; y < g_depthMD.YRes(); ++y) {
        const XnDepthPixel* pDepth = pDepthRow;
        XnRGB24Pixel* pTex = pTexRow + g_depthMD.XOffset();

        for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth, ++pTex) {
            if (*pDepth != 0) {
                int nHistValue = g_pDepthHist[*pDepth];
                pTex->nRed = nHistValue;
                pTex->nGreen = nHistValue;
                pTex->nBlue = 0;
            }
        }

        pDepthRow += g_depthMD.XRes();
        pTexRow += g_nTexMapX;
    }

    // Create the OpenGL texture map
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, g_pTexMap);

    // Display the OpenGL texture map
    glColor4f(1, 1, 1, 1);

    glBegin(GL_QUADS);

    int nXRes = g_depthMD.FullXRes();
    int nYRes = g_depthMD.FullYRes();

    // upper left
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    // upper right
    glTexCoord2f((float)nXRes/(float)g_nTexMapX, 0);
    glVertex2f(GL_WIN_SIZE_X, 0);
    // bottom right
    glTexCoord2f((float)nXRes / (float)g_nTexMapX, (float)nYRes / (float)g_nTexMapY);
    glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    // bottom left
    glTexCoord2f(0, (float)nYRes / (float)g_nTexMapY);
    glVertex2f(0, GL_WIN_SIZE_Y);

    glEnd();

    // Swap the OpenGL display buffers
    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y)
{
    // Press ESC key to exit.
    if (key == 27) exit(1);
}

int main(int argc, char* argv[])
{
    XnStatus rc;

    EnumerationErrors errors;
    rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
    if (rc == XN_STATUS_NO_NODE_PRESENT) {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (rc);
    } else if (rc != XN_STATUS_OK) {
        printf("Open failed: %s\n", xnGetStatusString(rc));
        return (rc);
    }

    rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
    if (rc != XN_STATUS_OK) {
        printf("No depth node exists! Check your XML.");
        return 1;
    }

    g_depth.GetMetaData(g_depthMD);

    // Texture map init
    g_nTexMapX = (((unsigned short)(g_depthMD.FullXRes()-1) / 512) + 1) * 512;
    g_nTexMapY = (((unsigned short)(g_depthMD.FullYRes()-1) / 512) + 1) * 512;
    g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

    g_nZRes = g_depthMD.ZRes();
    g_pDepthHist = (float*)malloc(g_nZRes * sizeof(float));

    // OpenGL init
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow ("simple-depth");
    glutSetCursor(GLUT_CURSOR_NONE);

    glutKeyboardFunc(keyboard);
    glutDisplayFunc(display);
    glutIdleFunc(idle);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    // Per frame code is in glutDisplay
    glutMainLoop();

    return 0;
}
