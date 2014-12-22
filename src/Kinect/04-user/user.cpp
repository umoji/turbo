#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>
#include <stdio.h>
#include <time.h>
// #include "ros/ros.h"
// #include "std_msgs/String.h"

#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
    #include <GLUT/glut.h>
#else
    #include <GL/glut.h>
#endif


#define SAMPLE_XML_PATH "../../config/SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)                                     \
    if (nRetVal != XN_STATUS_OK) {                                  \
        printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
        return nRetVal;                                             \
    }

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480   

double cog_x[14] = {};
double cog_y[14] = {};
double last_cog_x[14] = {};
double last_cog_y[14] = {};
int k = 0;
int userid = 0;

xn::Context g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bQuit = false;

//ユーザIDごとに付ける色の指定
XnFloat Colors[][3] =
{
    {0,1,1},
    {0,0,1},
    {0,1,0},
    {1,1,0},
    {1,0,0},
    {1,.5,0},
    {.5,1,0},
    {0,.5,1},
    {.5,0,1},
    {1,1,.5},
    {1,1,1}
};
XnUInt32 nColors = 10;


void glPrintString(void *font, char *str)
{
    int i, l = (int)strlen(str);

    for (i = 0; i < l; ++i) {
        glutBitmapCharacter(font, *str++);
    }
}

unsigned int getClosestPowerOfTwo(unsigned int n)
{
    unsigned int m = 2;
    while(m < n) m<<=1;

    return m;
}

GLuint initTexture(void** buf, int& width, int& height)
{
    GLuint texID = 0;
    glGenTextures(1, &texID);

    width = getClosestPowerOfTwo(width);
    height = getClosestPowerOfTwo(height);
    *buf = new unsigned char[width * height * 4];
    glBindTexture(GL_TEXTURE_2D, texID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return texID;
}

GLfloat texcoords[8];

void DrawRectangle(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
    GLfloat verts[8] = {
        topLeftX,     topLeftY,
        topLeftX,     bottomRightY,
        bottomRightX, bottomRightY,
        bottomRightX, topLeftY};
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    glFlush();
}

void DrawTexture(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

    DrawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY);

    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

void CleanupExit()
{
    g_scriptNode.Release();
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Context.Release();

    exit(0);
}

void DrawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd)
{
    static bool bInitialized = false;
    static GLuint depthTexID;
    static unsigned char* pDepthTexBuf;
    static int texWidth, texHeight;

    float topLeftX;
    float topLeftY;
    float bottomRightY;
    float bottomRightX;
    float texXpos;
    float texYpos;

    if(!bInitialized) {
        texWidth =  getClosestPowerOfTwo(dmd.XRes());
        texHeight = getClosestPowerOfTwo(dmd.YRes());

        depthTexID = initTexture((void**)&pDepthTexBuf, texWidth, texHeight) ;

        bInitialized = true;

        topLeftX = dmd.XRes();
        topLeftY = 0;
        bottomRightY = dmd.YRes();
        bottomRightX = 0;
        texXpos = (float)dmd.XRes() / texWidth;
        texYpos = (float)dmd.YRes() / texHeight;

        memset(texcoords, 0, 8*sizeof(float));
        texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;
    }

    unsigned int nValue = 0;
    unsigned int nHistValue = 0;
    unsigned int nIndex = 0;
    unsigned int nX = 0;
    unsigned int nY = 0;
    unsigned int nNumberOfPoints = 0;
    XnUInt16 g_nXRes = dmd.XRes();
    XnUInt16 g_nYRes = dmd.YRes();

    unsigned char* pDestImage = pDepthTexBuf;

    const XnDepthPixel* pDepth = dmd.Data();
    const XnLabel* pLabels = smd.Data();

    static unsigned int nZRes = dmd.ZRes();
    static float* pDepthHist = (float*)malloc(nZRes* sizeof(float));

    // Calculate the accumulative histogram
    memset(pDepthHist, 0, nZRes * sizeof(float));
    for (nY = 0; nY < g_nYRes; ++nY) {
        for (nX = 0; nX < g_nXRes; ++nX) {
            nValue = *pDepth;

            if (nValue != 0) {
                pDepthHist[nValue]++;
                nNumberOfPoints++;
            }

            pDepth++;
        }
    }

    for (nIndex = 1; nIndex < nZRes; ++nIndex) {
        pDepthHist[nIndex] += pDepthHist[nIndex - 1];
    }
    if (nNumberOfPoints) {
        for (nIndex = 1; nIndex < nZRes; ++nIndex) {
            pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (pDepthHist[nIndex] / nNumberOfPoints)));
        }
    }

    pDepth = dmd.Data();

    nIndex = 0;
    // Prepare the texture map
    for (nY = 0; nY < g_nYRes; ++nY) {
        for (nX = 0; nX < g_nXRes; ++nX, ++nIndex) {
            pDestImage[0] = 0;
            pDestImage[1] = 0;
            pDestImage[2] = 0;

            //pLabelsが0でない領域（ユーザが検出されている領域）に色を塗る
            if (g_bDrawBackground || *pLabels != 0) {
                nValue = *pDepth;
                XnLabel label = *pLabels;
                XnUInt32 nColorID = label % nColors;
                if (label == 0) {
                    nColorID = nColors;
                }

                if (nValue != 0) {
                    nHistValue = pDepthHist[nValue];

                    pDestImage[0] = nHistValue * Colors[nColorID][0];
                    pDestImage[1] = nHistValue * Colors[nColorID][1];
                    pDestImage[2] = nHistValue * Colors[nColorID][2];
                }
            }

            pDepth++;
            pLabels++;
            pDestImage += 3;
        }

        pDestImage += (texWidth - g_nXRes) * 3;
    }

    glBindTexture(GL_TEXTURE_2D, depthTexID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

    glColor4f(0.75, 0.75, 0.75, 1);

    glEnable(GL_TEXTURE_2D);
    DrawTexture(dmd.XRes(),dmd.YRes(),0,0);
    glDisable(GL_TEXTURE_2D);

    char strLabel[50] = "";
    XnUserID aUsers[15];
    XnUInt16 nUsers = 15;
    
    //認識中の人数の取得
    g_UserGenerator.GetUsers(aUsers, nUsers);
    for (int i = 0; i < nUsers; ++i) {//ユーザの人数分繰り返す
        //カメラを原点としたユーザの重心位置の取得
        XnPoint3D com;
        g_UserGenerator.GetCoM(aUsers[i], com);
        g_DepthGenerator.ConvertRealWorldToProjective(1, &com, &com);

        XnUInt32 nDummy = 0;

        xnOSMemSet(strLabel, 0, sizeof(strLabel));
        xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "user: %d", aUsers[i]);

        glColor4f(1 - Colors[i % nColors][0],
                  1 - Colors[i % nColors][1],
                  1 - Colors[i % nColors][2],
                  1);

        // 重心位置にラベルを描画
        if(k==30){
            if((last_cog_x[i]-com.X)*(last_cog_x[i]-com.X)+(last_cog_y[i]-com.Y)*(last_cog_y[i]-com.Y)>70||(last_cog_x[i]-com.X)*(last_cog_x[i]-com.X)+(last_cog_y[i]-com.Y)*(last_cog_y[i]-com.Y)<-70){
                printf("moved user:%d\n",i+1);
            }
            printf("(cog_x[%d],cog_y[%d]) = (%f,%f)\n",i+1 ,i+1 ,com.X ,com.Y);
            last_cog_x[i] = com.X;
            last_cog_y[i] = com.Y;
            }
        glRasterPos2i(com.X, com.Y);
        glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
    }
    if(k==30){
        k=0;
    }
}

// Callback: New user was detected
// ユーザ検出時のコールバック関数
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    printf("New user: %d\n", nId);
}

// Callback: An existing user was lost
// ユーザ消失時のコールバック関数
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    printf("Lost user: %d\n", nId);
}

void idle (void)
{
    if (g_bQuit) CleanupExit();
    glutPostRedisplay();
}

void display(void)
{
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // printf ("%d\n",i);

    // Setup the OpenGL viewpoint
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    xn::SceneMetaData sceneMD;
    xn::DepthMetaData depthMD;
    g_DepthGenerator.GetMetaData(depthMD);
    glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);

    glDisable(GL_TEXTURE_2D);

    g_Context.WaitOneUpdateAll(g_UserGenerator);

    // Process the data
    g_DepthGenerator.GetMetaData(depthMD);

    //ユーザデータの取得
    g_UserGenerator.GetUserPixels(0, sceneMD);
    DrawDepthMap(depthMD, sceneMD);

    glutSwapBuffers();
    if(k==30){
        k=0;
    }

    k++;

}

void keyboard (unsigned char key, int x, int y)
{
    switch (key) {
    // Press ESC key to exit.
    case 27:
        CleanupExit();
    case 'b':
        // Draw background?
        g_bDrawBackground = !g_bDrawBackground;
        break;
    }
}

void glInit (int *pargc, char **argv)
{
    glutInit(pargc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow ("user tracker");
    glutSetCursor(GLUT_CURSOR_NONE);

    glutKeyboardFunc(keyboard);
    glutDisplayFunc(display);
    glutIdleFunc(idle);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}

int main(int argc, char **argv)
{
// ----------------------------------------------------------------------
    // ros::init(argc, argv, "user");
    // ros::NodeHandle n;
    // ros::Publisher center_of_gravity_pub = n.advertise<std_msgs::String>("/pro/center_of_gravity", 1000);
    // ros::Rate loop_rate(10);
    // std_msgs::String msg;
    // std::stringstream cog;
    // cog << "center of gravity is [" << cog_x << "," << cog_y << "]";
    // msg.data = cog.str(); 
    // ROS_INFO("%s", msg.data.c_str());
    // ros::spinOnce();
    // loop_rate.sleep();
// ----------------------------------------------------------------------

    XnStatus nRetVal = XN_STATUS_OK;

    xn::EnumerationErrors errors;

    //XMLファイルを基にOpenNIコンテクストを開始
    nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
    if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    } else if (nRetVal != XN_STATUS_OK) {
        printf("Open failed: %s\n", xnGetStatusString(nRetVal));
        return (nRetVal);
    }

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    if (nRetVal != XN_STATUS_OK) {
        printf("No depth generator found. Using a default one...");
        xn::MockDepthGenerator mockDepth;
        nRetVal = mockDepth.Create(g_Context);
        CHECK_RC(nRetVal, "Create mock depth");

        // set some defaults
        XnMapOutputMode defaultMode;
        defaultMode.nXRes = 320;
        defaultMode.nYRes = 240;
        defaultMode.nFPS = 30;
        nRetVal = mockDepth.SetMapOutputMode(defaultMode);
        CHECK_RC(nRetVal, "set default mode");

        // set FOV
        XnFieldOfView fov;
        fov.fHFOV = 1.0225999419141749;
        fov.fVFOV = 0.79661567681716894;
        nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
        CHECK_RC(nRetVal, "set FOV");

        XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
        XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);

        nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
        CHECK_RC(nRetVal, "set empty depth map");

        g_DepthGenerator = mockDepth;
    }

    //ユーザジェネレータの作成
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK) {
        nRetVal = g_UserGenerator.Create(g_Context);
        CHECK_RC(nRetVal, "Find user generator");
    }

    //ユーザコールバックの登録
    XnCallbackHandle hUserCallbacks;
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");

    //データの作成を開始
    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    glInit(&argc, argv);
    glutMainLoop();
}
