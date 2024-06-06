#include "preproc.h"

void PreProc::Init()
{
    AlgoConfig::instance()->getImgSize(mImgWidth, mImgHeight);
    mPreResImg.Alloc(ipp::IwiSize(mImgWidth, mImgHeight), ipp32f, 1);
}

void PreProc::UnInit()
{
    if(mPreResImg.Exists()){
          mPreResImg.Release();
    }
}

void PreProc::Configure()
{
    std::string d2d_prepro_path = "/Inspection/Detection/D2D/PreProcess";
    std::string c2c_prepro_path = "/Inspection/Detection/C2C/PreProcess";
    pugi::xml_node prepro_node;
    DETECTION_MODE mode = AlgoConfig::instance()->GetDetectionMode();
    if(mode == DETECTION_MODE::D2D)
        prepro_node=AlgoConfig::instance()->SelectNodes(d2d_prepro_path);
    if(mode == DETECTION_MODE::C2C)
        prepro_node=AlgoConfig::instance()->SelectNodes(c2c_prepro_path);
    

    pugi::xml_node stretch_node = prepro_node.child("Stretch");
    mStretchLow=stretch_node.attribute("low").as_float();
    mStretchHigh=stretch_node.attribute("high").as_float();
    mGamma=stretch_node.attribute("gamma").as_float();

    pugi::xml_node smooth_node = prepro_node.child("Smooth");
    std::string smoothMethod_str = smooth_node.attribute("method").value();

    if(!strcmp(smoothMethod_str.c_str(), "Mean")){
      mSmoothMethod=Mean;
      mSmoothKSize= smooth_node.child("Mean").attribute("ksize").as_float();
    }
    else if(!strcmp(smoothMethod_str.c_str(), "Gauss")){
      mSmoothMethod=Gauss;
      mSmoothKSize= smooth_node.child("Gauss").attribute("ksize").as_float();
      mSmoothSigma= smooth_node.child("Gauss").attribute("sigma").as_float();
    }
    pugi::xml_node flatten_node = prepro_node.child("Flatten");
    std::string flattenMethod_str = flatten_node.attribute("method").value();
    std::string flattenFlag_str = flatten_node.attribute("flag").value();

    if(!strcmp(flattenFlag_str.c_str(), "true"))
       mFlattenFlag=true;
    else
       mFlattenFlag=false;

    if(!strcmp(flattenMethod_str.c_str(), "ButterWorth")){
       mFlattenMethod=ButterWorth;
       mOrder= flatten_node.child("ButterWorth").attribute("order").as_float();
       mFlattenSigma= flatten_node.child("ButterWorth").attribute("sigma").as_float();
    }
    else if(!strcmp(flattenMethod_str.c_str(), "Polyfit")){
       mFlattenMethod=Polyfit;
       mOrder= flatten_node.child("Polyfit").attribute("order").as_float();
    }

    pugi::xml_node locnorm_node = prepro_node.child("LocalNorm");
    mLocNormKSize=locnorm_node.attribute("ksize").as_float();
}

WRMat& PreProc::doPreproc(WRMat& src)
{
    WRMat temp1, temp2;
    temp1.Alloc(ipp::IwiSize(mImgWidth, mImgHeight), ipp8u, 1);
    temp2.Alloc(ipp::IwiSize(mImgWidth, mImgHeight), ipp8u, 1);
    histStretch(src, mStretchLow, mStretchHigh, temp1);
    switch(mSmoothMethod)
    {
        case SmoothMethod::Mean:
        {
          mean_8u(temp1, mSmoothKSize, temp2);
         }
        break;
        case SmoothMethod::Gauss:
        break;
        default:
        break;
    }
    if(mFlattenFlag==true)
    {
        switch(mFlattenMethod)
        {
            case FlattenMethod::ButterWorth:
            {
              butterWorth(temp2, mFlattenSigma, mOrder, temp1);
            }
            break;
            case FlattenMethod::Polyfit:
            break;
            default:
            break;
        }
        localNormalize(temp1, mLocNormKSize, mPreResImg);
    }
    else
    {
        localNormalize(temp2, mLocNormKSize, mPreResImg);
    }
    return mPreResImg;
}



