#include <stdio.h>
#include <Windows.h>
#include <process.h>
#include <conio.h>
#include "MvCameraControl.h"

bool g_bExit = false;
unsigned int g_DeviceKey = 1;
unsigned int g_GroupKey = 1;
unsigned int g_GroupMask= 1;

// ch:�ȴ��������� | en:Wait for key press
void WaitForKeyPress(void)
{
    while(!_kbhit())
    {
        Sleep(10);
    }
    _getch();
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:��ӡ��ǰ���ip���û��Զ������� | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

static  unsigned int __stdcall ActionCommandWorkThread(void* pUser)
{
    int nRet = MV_OK;
    MV_ACTION_CMD_INFO stActionCmdInfo = {0};
    MV_ACTION_CMD_RESULT_LIST stActionCmdResults = {0};

    stActionCmdInfo.nDeviceKey = g_DeviceKey;
    stActionCmdInfo.nGroupKey = g_GroupKey;
    stActionCmdInfo.nGroupMask = g_GroupMask;
    stActionCmdInfo.pBroadcastAddress = "255.255.255.255";
    stActionCmdInfo.nTimeOut = 100;
    stActionCmdInfo.bActionTimeEnable = 0;

    while(!g_bExit)
    {
        //Send the PTP clock photo command
        nRet = MV_GIGE_IssueActionCommand(&stActionCmdInfo,&stActionCmdResults);
        if (MV_OK != nRet)
        {
            printf("Issue Action Command fail! nRet [0x%x]\n", nRet);
            continue;
        }
        printf("NumResults = %d\r\n",stActionCmdResults.nNumResults);

        MV_ACTION_CMD_RESULT* pResults = stActionCmdResults.pResults;
        for (unsigned int i = 0;i < stActionCmdResults.nNumResults;i++)
        {
            //Print the device infomation
            printf("Ip == %s, Status == 0x%x\r\n",pResults->strDeviceAddress,pResults->nStatus);
            pResults++;
        }
    }

    return 0;
}

static  unsigned int __stdcall ReceiveImageWorkThread(void* pUser)
{
    int nRet = MV_OK;
    MV_FRAME_OUT stImageInfo = {0};

    while(1)
    {
        nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            printf("Get Image Buffer: Width[%d], Height[%d], FrameNum[%d]\n", 
                stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nFrameNum);

            nRet = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
            if(nRet != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
        }
        else
        {
            printf("Get Image fail! nRet [0x%x]\n", nRet);
        }
        if(g_bExit)
        {
            break;
        }
    }

    return 0;
}

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;

    do 
    {
        // ch:ö���豸 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum-1);
        unsigned int nIndex = 0;
        scanf_s("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Input error!\n");
            break;
        }

        // ch:ѡ���豸��������� | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:���豸 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:̽��������Ѱ���С(ֻ��GigE�����Ч) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // ch:���ô���ģʽΪon | en:Set trigger mode as on
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:���ô���ԴΪAction1 | en:Set trigger source as Action1
        nRet = MV_CC_SetEnumValueByString(handle, "TriggerSource", "Action1");
        if (MV_OK != nRet)
        {
            printf("Set Trigger Source fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:����Action Device Key | en:Set Action Device Key
        nRet = MV_CC_SetIntValue(handle, "ActionDeviceKey", g_DeviceKey);
        if (MV_OK != nRet)
        {
            printf("Set Action Device Key fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:����Action Group Key | en:Set Action Group Key100
        nRet = MV_CC_SetIntValue(handle, "ActionGroupKey", g_GroupKey);
        if (MV_OK != nRet)
        {
            printf("Set Action Group Key fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:����Action Group Mask | en:Set Action Group Mask
        nRet = MV_CC_SetIntValue(handle, "ActionGroupMask", g_GroupMask);
        if (MV_OK != nRet)
        {
            printf("Set Action Group Mask fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:��ʼȡ�� | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        unsigned int nReceiveImageThreadID = 0;
        void* hReceiveImageThreadHandle = (void*) _beginthreadex( NULL , 0 , ReceiveImageWorkThread , handle, 0 , &nReceiveImageThreadID );
        if (NULL == hReceiveImageThreadHandle)
        {
            break;
        }

        unsigned int nActionCommandThreadID = 0;
        void* hActionCommandThreadHandle = (void*) _beginthreadex( NULL , 0 , ActionCommandWorkThread , NULL, 0 , &nActionCommandThreadID );
        if (NULL == hActionCommandThreadHandle)
        {
            return 0;
        }

        printf("Press a key to stop grabbing.\n");
        WaitForKeyPress();

        g_bExit = true;
        Sleep(1000);

        // ch:ֹͣȡ�� | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:�ر��豸 | Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("ClosDevice fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:���پ�� | Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
    } while (0);
    

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("Press a key to exit.\n");
    WaitForKeyPress();

    return 0;
}
