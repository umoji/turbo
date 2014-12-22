#include <math.h>
#include <XnOS.h>
#include <XnCppWrapper.h>

#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
    #include <GLUT/glut.h>
#else
    #include <GL/glut.h>
#endif

using namespace xn;

#define ROS_MODE 1 // 1 when using ROS

#define SAMPLE_XML_PATH "../../config/SamplesConfig.xml"

#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480


XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;

Context g_context;
ScriptNode g_scriptNode;
ImageGenerator g_image;
ImageMetaData g_imageMD;


void idle(void)
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

    //イメージメタデータの取得
    g_image.GetMetaData(g_imageMD);

    // Clear the OpenGL buffers
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Setup the OpenGL viewpoint
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

    xnOSMemSet(g_pTexMap, 0, g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

#if !ROS_MODE
    // check if we need to draw image frame to texture
    const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
    XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

    for (XnUInt y = 0; y < g_imageMD.YRes(); ++y) {
        const XnRGB24Pixel* pImage = pImageRow;
        XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();

        for (XnUInt x = 0; x < g_imageMD.XRes(); ++x, ++pImage, ++pTex) {
            *pTex = *pImage;
        }

        pImageRow += g_imageMD.XRes();
        pTexRow += g_nTexMapX;
    }
#else
    // check if we need to draw image frame to texture
    //const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
    const XnUInt8* pImageRow = g_imageMD.Data();
    const XnUInt8* pImageRow1 = g_imageMD.Data();
    XnRGB24Pixel* pTexRow = g_pTexMap + g_imageMD.YOffset() * g_nTexMapX;

    for (XnUInt y = 0; y < g_imageMD.YRes(); ++y) {
        //const XnRGB24Pixel* pImage = pImageRow;
        const XnUInt8* pImage = pImageRow;
        const XnUInt8* pImage1 = pImageRow1;
        XnRGB24Pixel* pTex = pTexRow + g_imageMD.XOffset();

        for (XnUInt x = 0; x < g_imageMD.XRes();) {
            // blue line
            //                *pTex = *pImage;
            XnRGB24Pixel rgb;
            static XnUInt8 r, g, b;
            if (y % 2 == 0) {
                if ( x % 2 == 0 ) {
                    g = *pImage++;
                    b = *pImage1++;
                } else {
                    r = *pImage++;
                    *pImage1++;
                }
            } else {
                if (x % 2 == 0) {
                    b = *pImage++;
                    *pImage1++;
                } else {
                    g = *pImage++;
                    r = *pImage1++;
                }
            }

            rgb.nRed = r;
            rgb.nBlue = b;
            rgb.nGreen = g;
            *pTex = rgb;
            ++pTex;

            ++x;
        }

        pImageRow1 = pImageRow;
        pImageRow += g_imageMD.XRes();
        pTexRow += g_nTexMapX;
    }
#endif

    // Create the OpenGL texture map
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, g_pTexMap);

    // Display the OpenGL texture map
    glColor4f(1, 1, 1, 1);

    glBegin(GL_QUADS);

    int nXRes = g_imageMD.FullXRes();
    int nYRes = g_imageMD.FullYRes();

    // upper left
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    // upper right
    glTexCoord2f((float)nXRes / (float)g_nTexMapX, 0);
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

void keyboard(unsigned char key, int /*x*/, int /*y*/)
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

    //ImageGeneratorの生成
    rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
    if (rc != XN_STATUS_OK) {
        printf("No image node exists! Check your XML.");
        return 1;
    }

    g_image.GetMetaData(g_imageMD);

#if !ROS_MODE
    // RGB is the only image format supported.
    if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24) {
      printf("The device image format must be RGB24 %d\n",g_imageMD.PixelFormat());
      return 1;
    }
#endif

    // Texture map init
    g_nTexMapX = (((unsigned short)(g_imageMD.FullXRes() - 1) / 512) + 1) * 512;
    g_nTexMapY = (((unsigned short)(g_imageMD.FullYRes() - 1) / 512) + 1) * 512;
    g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

    // OpenGL init
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow ("simple-image");
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
