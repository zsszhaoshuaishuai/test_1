#ifndef _ALIGNMENT_H_
#define _GLIGNMENT_H_
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "MeasureInstance.h"

typedef enum {
    AlignX,
    AlignY,
}AlignDirection;
typedef enum
{
    Line=0,
    Hole,
    Unique,
    i_FEM,
    Entire
}PatternMode;
typedef enum
{
    NCC = 0,
    PHOT
}TemplateMethod;

class Alignment
{
public:
    Alignment();
    ~Alignment();

protected:
    Alignment(const Alignment&) = delete;
public:
    CDError::ECode configure(cv::Mat& ref, cv::Mat &test);
    CDError::ECode calcProfile(cv::Mat& src, cv::Mat& profile);
    CDError::ECode phaseCorrelation(cv::Mat& tpl, cv::Mat& img, std::vector<cv::Point2f>& positions);
    CDError::ECode templateMatch1D(cv::Mat& tpl, cv::Mat& img, int accept, cv::Mat& result_mat, std::vector<cv::Point2f>& positions);
    CDError::ECode templateMatch2D(cv::Mat& tpl, cv::Mat& img, int accept, cv::Mat& result_mat, std::vector<cv::Point2f>& positions);
    CDError::ECode doAlgnment();
    void getRelativeShift(float& offsetX, float& offsetY);
    void debugImg(cv::Mat& test, cv::Point2f offSet, cv::Mat& shiftImg);
    void calcuAlignScoreNCC(cv::Mat& test, cv::Mat tmpl, cv::Point2f offset, double& score);
    int checkScore(double score);

public:
    cv::Mat mRefImg, mTemplImg, mTestImg;  
private:
    int mX, mY, mWidth, mHeight;
    float mOffsetX, mOffsetY;
    AlignDirection mDirection;
    TemplateMethod mTemplMethod;
    PatternMode mPatternMode;
    cv::Mat mTemplResult;
    std::vector<cv::Point2f> mPositions;
    int mAcceptance;
    float mThreshold;
};

#endif
