#include <cstdlib>
#include <cmath>

#include "Alignment.h"
#include "ClassFactory.h"
#include "CDConfig.h"
#include "log.hpp"
#include "Error.h"
#include "pugixml.hpp"
#include "ImgProcLibs.h"
#include "findpeaks.h"
#include "profileOperation.h"
#include "MathLibs.h"
#include <iostream>


Alignment::Alignment()
{
    mThreshold = 0.8;
};

Alignment::~Alignment()
{

};
CDError::ECode Alignment::configure(cv::Mat& ref,cv::Mat& test)
{
    CDError::ECode err = CDERR_OK;
    //get template image
    mTestImg = test.clone();
    mTestImg.convertTo(mTestImg, CV_32F);

    pugi::xml_node gauge_node = CDConfig::instance()->getCurrentGauge();
    pugi::xml_node alignment_node = gauge_node.child("Alignment");
    pugi::xml_node measurementArea_node = gauge_node.child("MeasurementArea");
    std::string strDirection = measurementArea_node.child_value("Direction");
    std::string pattmode_str = alignment_node.child_value("PatternMode");

    if (!strcmp(pattmode_str.c_str(), "Line"))
        mPatternMode = PatternMode::Line;
    else if(!strcmp(pattmode_str.c_str(), "Hole"))
        mPatternMode = PatternMode::Hole;
    else if(!strcmp(pattmode_str.c_str(), "Unique"))
        mPatternMode = PatternMode::Unique;
    else if (!strcmp(pattmode_str.c_str(), "i_FEM"))
        mPatternMode = PatternMode::i_FEM;
    else 
        mPatternMode = PatternMode::Entire;
   
    if (strDirection == "X")
        mDirection = AlignX;
    else
        mDirection = AlignY;  
    //measurement area  
    mX= alignment_node.child("Position").attribute("X").as_float();
    mY = alignment_node.child("Position").attribute("Y").as_float();
    mWidth = alignment_node.child("Position").attribute("Width").as_float();
    mHeight = alignment_node.child("Position").attribute("Height").as_float();
    mAcceptance = atof(alignment_node.child_value("Acceptance"));

    if (mPatternMode == PatternMode::Entire){
        mTemplImg = ref;
    }
    else{
        cv::Rect rect(mX, mY, mWidth, mHeight);
        mTemplImg = ref(rect);      
    }
    mTemplImg.convertTo(mTemplImg,CV_32F);
    std::string template_method = alignment_node.child_value("TemplateMethod");
    if (!strcmp(template_method.c_str(), "NCC"))
        mTemplMethod = TemplateMethod::NCC;
    else
        mTemplMethod = TemplateMethod::PHOT;
    return err;
}
CDError::ECode Alignment::calcProfile(cv::Mat& src, cv::Mat& profile)
{
    CDError::ECode err = CDERR_OK;
    if(mDirection==AlignDirection::AlignX)
        cv::reduce(src, profile, 0, cv::REDUCE_AVG, CV_32F);
    if (mDirection == AlignDirection::AlignY){
        cv::reduce(src, profile, 1, cv::REDUCE_AVG, CV_32F);
        cv::transpose(profile, profile);
    }
    return err;
}
CDError::ECode Alignment::templateMatch1D(cv::Mat& tpl, cv::Mat& img, int accept, cv::Mat& result_mat, std::vector<cv::Point2f>& positions)
{
    CDError::ECode err = CDERR_OK;
    cv::Mat testProfile, tmplProfile;
    calcProfile(tpl, tmplProfile);
    calcProfile(img, testProfile);
    int match_method = cv::TM_CCORR_NORMED;
    cv::matchTemplate(testProfile, tmplProfile, result_mat, match_method);
    cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    //find the domiant peaks
    double minVal; double maxVal;
    cv::Point minLoc, maxLoc, matchLoc;
    cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    int tplW = tmplProfile.cols;
    int dilation_size =(std::min)(16, tplW / 2);  //default min peak distance
    cv::Mat dilation_mat, erosion_mat;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilation_size, 1));
    dilate(result_mat, dilation_mat, element);
    erode(result_mat, erosion_mat, element);
    cv::Mat diff0, diff1;
    diff0 = (result_mat - dilation_mat) > -1e-6;
    float dominate_th = 0.3f;
    diff1 = (dilation_mat - erosion_mat) > dominate_th * maxVal;

    std::vector<cv::Point> idx;
    bitwise_and(diff0, diff1, diff0);
    cv::findNonZero(diff0, idx);

    std::vector<cv::Point> tempPositions;
    tempPositions.clear();
    for (int i = 0; i < idx.size(); i++) {
        if (result_mat.at<float>(idx[i].y, idx[i].x) > mThreshold * maxVal) {
            tempPositions.push_back(idx[i]);
        }
    }
    if (tempPositions.size() <= 0) {
        LOG_ERR("templateMatch2D method match error ");
        throw CDError("templateMatch2D method match error ", CDERR_AlignmentFailed);
    }
    std::vector<cv::Point>::iterator iter = tempPositions.begin();
    //find latest similar to template center: center= mX+mWidth/2
    int itemp = 0;
    if (mDirection == AlignDirection::AlignX) {
        int dTemp = fabs((*iter).x - mX);
        
        for (int ix = 0; iter != tempPositions.end(); ++iter, ++ix)
        {
            if ((dTemp > fabs((*iter).x - mX)) || (dTemp == fabs((*iter).x - mX)))
            {
                dTemp = fabs((*iter).x - mX);
                itemp = ix;
            }
        }      
    }
    //find latest similar to template center: center= mY+mHeight/2
    if (mDirection == AlignDirection::AlignY) {
        int dTemp = fabs((*iter).x - mY);
        for (int ix = 0; iter != tempPositions.end(); ++iter, ++ix)
        {
            if ((dTemp > fabs((*iter).x - mY)) || (dTemp == fabs((*iter).x - mY)))
            {
                dTemp = fabs((*iter).x - mY);
                itemp = ix;
            }
        }
        int temp = 0;
        temp = tempPositions[itemp].x;
        tempPositions[itemp].x = tempPositions[itemp].y;
        tempPositions[itemp].y = temp;
    } 
   
    if (mDirection == AlignDirection::AlignX){
        tempPositions[itemp].x = tempPositions[itemp].x -mX;
    }
    else{
        tempPositions[itemp].y = tempPositions[itemp].y -mY;
    }
    positions.push_back(tempPositions[itemp]);
    return err;
}
CDError::ECode Alignment::templateMatch2D(cv::Mat& tpl, cv::Mat& img, int accept, cv::Mat& result_mat, std::vector<cv::Point2f>& positions)
{
    CDError::ECode err = CDERR_OK;
    int match_method = cv::TM_CCORR_NORMED;
    cv::Mat aa = tpl;
    aa.convertTo(aa, CV_8UC1);
    cv::matchTemplate(img, tpl, result_mat, match_method);
    cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    //find the domiant peaks
    double minVal; double maxVal;
    cv::Point minLoc, maxLoc, matchLoc;
    cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    int tplH = tpl.rows;
    int tplW = tpl.cols;
    int dilation_size =  (std::min)(16, tplW / 2);  //default min peak distance
    cv::Mat dilation_mat, erosion_mat;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilation_size, dilation_size));
    dilate(result_mat, dilation_mat, element);
    erode(result_mat, erosion_mat, element);
    cv::Mat diff0, diff1;
    diff0 = (result_mat - dilation_mat) > -1e-6;
    float dominate_th = 0.3f;
    diff1 = (dilation_mat - erosion_mat) > dominate_th * maxVal;

    std::vector<cv::Point> idx;
    idx.clear();
    bitwise_and(diff0, diff1, diff0);
    cv::findNonZero(diff0, idx);
    std::vector<cv::Point> tempPositions;
    tempPositions.clear();

    switch (mPatternMode)
    {
        case PatternMode::Unique:
        {
            for (int i = 0; i < idx.size(); i++) {
                if (result_mat.at<float>(idx[i].y, idx[i].x) > mThreshold * maxVal) {
                    tempPositions.push_back(idx[i]);
                }
            }
            if(tempPositions.size()<=0){
                LOG_ERR("templateMatch2D method match error ");
                    throw CDError("templateMatch2D method match error ", CDERR_AlignmentFailed);
            }
            float tempScore = 0;
            int tempId = 0;
            //judge the maxmimum value of score
            for (int j = 0; j < tempPositions.size(); j++)
            {
                if (result_mat.at<float>(tempPositions[j].y, tempPositions[j].x) > tempScore)
                {
                    tempScore = result_mat.at<float>(tempPositions[j].y, tempPositions[j].x);
                    tempId = j;
                }
            }
            tempPositions[tempId].x = tempPositions[tempId].x - mX;
            tempPositions[tempId].y = tempPositions[tempId].y - mY;
            positions.push_back(tempPositions[tempId]);
        }
        break;
        case PatternMode::Hole:
        case PatternMode::i_FEM:
        {
            for (int i = 0; i < idx.size(); i++) {
                if (result_mat.at<float>(idx[i].y, idx[i].x) > mThreshold * maxVal) {
                    tempPositions.push_back(idx[i]);
                }
            }
            if (tempPositions.size() <= 0) {
                LOG_ERR("templateMatch2D method match error ");
                throw CDError("templateMatch2D method match error ", CDERR_AlignmentFailed);
            }
            std::vector<cv::Point>::iterator iter = tempPositions.begin();
            //find latest similar to template x
            double dTemp = fabs(((*iter).x - mX) * ((*iter).x - mX) + ((*iter).y - mY) * ((*iter).y - mY));
            int itemp = 0;
            for (int ix = 0; iter != tempPositions.end(); ++iter, ++ix)
            {
                if ((dTemp >= fabs(((*iter).x - mX) * ((*iter).x - mX) + ((*iter).y - mY) * ((*iter).y - mY))))
                {
                    dTemp = fabs(((*iter).x - mX) * ((*iter).x - mX) + ((*iter).y - mY) * ((*iter).y - mY));
                    itemp = ix;
                }
            }
            tempPositions[itemp].x -= mX, tempPositions[itemp].y -= mY;
            positions.push_back(tempPositions[itemp]);
        }
        break;
        default:
            LOG_INFO("templateMatch2D default");
            break;
    }
    return err;
}
CDError::ECode Alignment::phaseCorrelation(cv::Mat& tpl, cv::Mat& img, std::vector<cv::Point2f>& positions)
{
    CDError::ECode err = CDERR_OK;
    positions.clear();
    cv::Point2f phase_shift;
    double score;
    cv::Mat noArray;
    phase_shift = cv::phaseCorrelate(tpl, img, noArray, &score);
    positions.push_back(phase_shift);
    return err;
}
CDError::ECode Alignment::doAlgnment()
{
    CDError::ECode err = CDERR_OK;
    mPositions.clear();
    switch (mPatternMode)
    { 
        case PatternMode::Line:
            err=templateMatch1D(mTemplImg, mTestImg, mAcceptance, mTemplResult, mPositions);
        break;
        case PatternMode::Hole:
        case PatternMode::i_FEM:
        case PatternMode::Unique:
            err=templateMatch2D(mTemplImg, mTestImg, mAcceptance, mTemplResult, mPositions);
        break;
        case PatternMode::Entire:
            err = phaseCorrelation(mTemplImg, mTestImg, mPositions);
        break;
        default:
            err = phaseCorrelation(mTemplImg, mTestImg, mPositions);
        break;
    }
    LOG_INFO("Alignment offset X: ", mPositions[0].x, "  Alignment offset Y: ", mPositions[0].y);
    return err;
}
void Alignment::getRelativeShift(float& offsetX, float& offsetY)
{
  offsetX = mPositions[0].x;
  offsetY = mPositions[0].y;
}

void Alignment::debugImg(cv::Mat& test, cv::Point2f offSet, cv::Mat& shiftImg)
{
    cv::Mat temp;
    temp = test.clone();
    int width = test.cols, height = test.rows;
    cv::Point2f center(width/2.0, height/2.0);
    cv::Size size(width, height);
    
    cv::Mat M = cv::getRotationMatrix2D(center, 0, 1.0);
    M.at<double>(0, 2) -= offSet.x;
    M.at<double>(1, 2) -= offSet.y;
    cv::warpAffine(temp, shiftImg, M, size);
}
void Alignment::calcuAlignScoreNCC(cv::Mat& test, cv::Mat tmpl, cv::Point2f offset, double& score)
{
    offset.x = ceil(offset.x), offset.y=ceil(offset.y);
    cv::Rect rect1, rect2;
    rect1.x = std::max(0.0f, offset.x), rect1.y = std::max(0.0f, offset.y);
    rect1.width = test.cols - std::abs(offset.x), rect1.height = test.rows - std::abs(offset.y);
    if (offset.x <= 0) {
        rect2.x = abs(offset.x);
    }
    else {
        rect2.x = 0;
    }    
    if (offset.y <= 0) {
        rect2.y = abs(offset.y);
    }
    else {
        rect2.y = 0;
    }      
    rect2.width = rect1.width, rect2.height = rect1.height;
    cv::Mat result;
    cv::matchTemplate(test(rect1), tmpl(rect2), result, cv::TM_CCOEFF_NORMED);
    score = result.at<float>(0, 0);
}
int Alignment::checkScore(double score)
{
    if (score < 0)
        score = 0;
    int nScore = int(score* 1000);
    if (nScore < mAcceptance) {
        LOG_ERR("Alignment score is less than acceptance");
        throw CDError("Alignment score is less than acceptance", CDERR_AlignmentFailed);
    }
    LOG_INFO("Alignment score is ",nScore);
    return nScore;
}




