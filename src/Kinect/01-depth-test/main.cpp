#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>


#define SAMPLE_XML_PATH "../../config/SamplesConfig.xml"

#define CHECK_RC(rc, what)                                          \
    if (rc != XN_STATUS_OK) {                                       \
        printf("%s failed: %s\n", what, xnGetStatusString(rc));     \
        return rc;                                                  \
    }


using namespace xn;

XnBool fileExists(const char *fn)
{
    XnBool exists;
    xnOSDoesFileExist(fn, &exists);
    return exists;
}

int main()
{
    XnStatus nRetVal = XN_STATUS_OK;

    Context context;
    ScriptNode scriptNode;
    EnumerationErrors errors;

    const char *fn = NULL;
    if  (fileExists(SAMPLE_XML_PATH)) {
        fn = SAMPLE_XML_PATH;
    } else {
        printf("Could not find '%s'. Aborting.\n" , SAMPLE_XML_PATH);
        return XN_STATUS_ERROR;
    }
    printf("Reading config from: '%s'\n", fn);
    nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

    if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    } else if (nRetVal != XN_STATUS_OK) {
        printf("Open failed: %s\n", xnGetStatusString(nRetVal));
        return (nRetVal);
    }

    //DepthGeneratorの生成
    DepthGenerator depth;
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
    CHECK_RC(nRetVal, "Find depth generator");

    XnFPSData xnFPS;
    nRetVal = xnFPSInit(&xnFPS, 180);
    CHECK_RC(nRetVal, "FPS Init");

    DepthMetaData depthMD;

    while (!xnOSWasKeyboardHit()) {
        nRetVal = context.WaitOneUpdateAll(depth);
        if (nRetVal != XN_STATUS_OK) {
            printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
            continue;
        }

        xnFPSMarkFrame(&xnFPS);

        //デプスメタデータの取得
        depth.GetMetaData(depthMD);

        //カメラ中心のピクセルに写った物体までの距離を計測して表示
        printf("Frame %d Middle point is: %u. FPS: %f\n", depthMD.FrameID(), depthMD(depthMD.XRes() / 2, depthMD.YRes() / 2), xnFPSCalc(&xnFPS));
    }

    depth.Release();
    scriptNode.Release();
    context.Release();

    return 0;
}
