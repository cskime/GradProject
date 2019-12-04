#include "AP_Camera.h"

/* Using Topics
 * 1. geometry_msgs/Point : point msgs
 * 2. std_msgs/Bool : boolean msgs
 */

//헤더에서 ros, topic 관련 라이브러리 추가
//#include <geometry_msgs/Point.h>
//#include <std_msgs/Bool.h>


/* Timer class
 */
SC_Timer::SC_Timer()
{
    timer = node_.createTimer(ros::Duration(0.1), &SC_Timer::timer_callback, this);
    isCounted = false;
    sec_start = 0;
    sec_flag = 0;
    sec_count = 100;
}

void SC_Timer::set_timer(int th_sec)
{
    if(sec_flag==0)
    {
        if(sec_start == 0)
        {
            sec_start = sec_count;
            sec_flag = 1;
        }
    }
    doing_t =th_sec;

}

void SC_Timer::counting(void)
{
    if(sec_flag !=0)
    {
        if(sec_count != sec_start)
        {
            sec_flag = sec_count - sec_start;
        }
        if(sec_flag >= doing_t)
        {
            isCounted = false;
            sec_flag = 0;
            sec_start = 0;
        }
        else
            isCounted = true;
        //cout << "sec_flag: " << sec_flag << "   sec_count: " << sec_count << "  sec_start: "<< sec_start<< endl;
    }
}

void SC_Timer::exit()
{
    isCounted = false;
    sec_flag = 999999;
}

void SC_Timer::timer_callback(const ros::TimerEvent& )
{
    //std::cout << "Crosswalk stop line check.." << std::endl;
    sec_count++;

}


/* Subscriber Callback */
// Front
void CCamera::subImgCallback1(const sensor_msgs::Image& subImgMsgs)
{
    if(subImgMsgs.data.size())
    {
        rawImagePtr1 = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImagefront = rawImagePtr1->image;

        rawImagePtr1->image = rawImagefront;
        //pubImage.publish(rawImagePtr->toImageMsg());
    }
}

// Side
void CCamera::subImgCallback2(const sensor_msgs::Image& subImgMsgs)
{
    if(subImgMsgs.data.size())
    {
        rawImagePtr2 = cv_bridge::toCvCopy(subImgMsgs, sensor_msgs::image_encodings::BGR8);
        rawImageside = rawImagePtr2->image;

        rawImagePtr2->image = rawImageside;
        //pubImage.publish(rawImagePtr->toImageMsg());
    }
}


/* Parking class
 */
CCamera::CCamera()
{
    m_wayPoint = Point2f(0.0f,0.0f);
    m_parkingSign = false;
    m_parkingArea = false;
    m_endParking = false;
    m_parkingReverse = false;
    m_endReverse = false;

    // +0.02, -0.02
    min_left = 2.20; max_left = 2.24;
    min_right = 1.79; max_right = 1.81;
    min_up = 1.21; max_up = 1.25;
    min_bottom = 0.20; max_bottom = 0.40;

    // flag vari
    wayX.data = 320; // image center
    isParkingSign.data = false;
    isParkingArea.data = false;
    isEndParking.data = false;
    isReverse.data = false;
    isEndReverse.data = false;

    subFrontImage = nh.subscribe("/camera1/usb_cam1/image_raw", 1, &CCamera::subImgCallback1, this);
    subParkingImage = nh.subscribe("/camera2/usb_cam2/image_raw", 1, &CCamera::subImgCallback2, this);

    // flag pub
    pub_waypointY = nh.advertise<std_msgs::Float32>("/Camera/wayPointY", 1);
    pub_parkingSign = nh.advertise<std_msgs::Bool>("/Camera/traffic/parking", 1);
    pub_parkingArea = nh.advertise<std_msgs::Bool>("/Camera/parking", 1);
    pub_endParking = nh.advertise<std_msgs::Bool>("/Camera/endParking", 1);
    pub_pReverse = nh.advertise<std_msgs::Bool>("/Camera/pReverse", 1);
    pub_endReverse = nh.advertise<std_msgs::Bool>("/Camera/endReverse", 1);
}

void CCamera::Processing()
{
    m_front = rawImagefront;
    m_side = rawImageside;

    frontProcessing();
    sideProcessing();
}

void CCamera::frontProcessing()
{
    cvtColor(m_front, m_gray, CV_BGR2GRAY);

    threshold(m_gray, m_binary, 0, 255, CV_THRESH_OTSU);

    // binary_reverse
    for (int i = 0; i < m_binary.rows; i++)
    {
        for (int j = 0; j < m_binary.cols; j++)
        {
            m_binary.at<uchar>(i, j) = 255 - m_binary.at<uchar>(i, j);
        }
    }

    //erode(binary, binary, getStructuringElement((MORPH_ELLIPSE), Size(3, 3)));
    dilate(m_binary, m_binary, getStructuringElement((MORPH_ELLIPSE), Size(3, 3)));

    int Line_L_X = 0;
    int Line_R_X = m_binary.cols - 1;
    float line_width = 0;

    for (int y = 390; y > 320; y--)
    {
        /////////////////////////Left Lane////////////////////////
        for (int x = (m_binary.cols / 2) - 1; x >= 0; x--)
        {
            uchar P = m_binary.at<uchar>(y, x);

            if (P == 255)
            {
                if (Line_L_X < x)
                {
                    Line_L_X = x;
                }
            }
        }

        /////////////////////////Right Lane////////////////////////
        for (int x = (m_binary.cols / 2) - 1; x < m_roi.br().x; x++)
        {
            uchar P = m_binary.at<uchar>(y, x);

            if (P == 255)
            {
                if (Line_R_X > x)
                {
                    Line_R_X = x;
                }
            }
        }

        if(Line_L_X != 0 && Line_R_X != (m_binary.cols-1) && (Line_R_X-Line_L_X) > 150)
        {
            m_wayPoint = Point((Line_L_X + Line_R_X)/2, y);
            line_width = Line_R_X-Line_L_X;
            //cout << "wayMID.x: " << wayMID.x << "  wayMID.y: " << wayMID.y << " Line_L_X:" << Line_L_X << "    Line_R_X:" << Line_R_X << "  line_width: "<< line_width << " Line_R_X-Line_L_X_white:" << Line_R_X-Line_L_X_white << endl;
            circle(m_binary, m_wayPoint, 5, Scalar(255), 5);
            break;
        }
    }

    wayX.data = m_wayPoint.x;
    pub_waypointY.publish(wayX);

    cout << m_wayPoint.x << endl;

    imshow("front", m_binary);
}

void CCamera::sideProcessing()
{
    findSign();
    findParkingArea();
}

void CCamera::findSign()
{
    m_roi = Rect(m_side.cols * 2 / 3, m_side.rows * 2 / 5, m_side.cols / 3 , m_side.rows * 3 / 5);

    Mat HSV_img;
    cvtColor(m_side(m_roi), HSV_img, CV_BGR2HSV);

    Mat tmp_blue, tmp_whi;
    Mat binary_blue, binary_white;

    inRange(HSV_img, Scalar(108, 58, 30), Scalar(123, 216, 150), tmp_blue);
    dilate(tmp_blue, binary_blue, Mat(),Point(-1,-1),2);

    inRange(HSV_img, Scalar(0, 0, 70), Scalar(255, 40, 243), tmp_whi);
    dilate(tmp_whi, binary_white, Mat(),Point(-1,-1),2);

    Mat img_labels, stats, centroids;
    int numOfLables = connectedComponentsWithStats(binary_blue, img_labels, stats, centroids, 8, CV_32S);

    int left, top, width, height;
    float div;

    // Most Big Labeling
    int max_b = -1, idx = 0;

    for (int j = 1; j < numOfLables; j++)
    {
        int area = stats.at<int>(j, CC_STAT_AREA);
        if (max_b < area)
        {
            max_b = area;
            idx = j;
        }
    }

    if(idx!=0)
    {
        left = stats.at<int>(idx, CC_STAT_LEFT);
        top = stats.at<int>(idx, CC_STAT_TOP);
        width = stats.at<int>(idx, CC_STAT_WIDTH);
        height = stats.at<int>(idx, CC_STAT_HEIGHT);

        if(height!=0)
            div = (float)width/height;
        else div = 0;

        if(div<0.8 || div > 1.3)
        {
            left = 0;
            top = 0;
            width = 0;
            height = 0;
        }
    }

    else
    {
        left = 0;
        top = 0;
        width = 0;
        height = 0;
    }

    if(max_b!= -1)
    {
        //cout << "bluemax: " << max_b << endl;
    }

    Mat whiROI = binary_white;
    int max_w = -1;

    for (int y = 0; y < binary_white.rows; y++)
    {
        for (int x = 0; x <binary_white.cols ; x++)
        {
            if(y>top && y<(top+height) && x>left && x<(left+width) && binary_white.at<uchar>(y,x) == 255)
            {
                whiROI.at<uchar>(y, x) = 255;
            }

            else
            {
                whiROI.at<uchar>(y, x) = 0;
            }
        }
    }

    int numOfLablesWhiBlue = connectedComponentsWithStats(whiROI, img_labels, stats, centroids, 8, CV_32S);

    for (int j = 1; j < numOfLablesWhiBlue; j++)
    {
        int area = stats.at<int>(j, CC_STAT_AREA);
        if (max_w < area)
        {
            max_w = area;
            idx = j;
        }
    }

    if(max_w != -1)
    {
        //cout << "whimax: " << max_w << endl;
    }

    if (max_w > 1000 && max_b > 2000)
    {
         // if(top + height < image.cols * 0.8)
        {
            m_parkingSign = true;
            rectangle(m_side, Point(left, top), Point(left + width, top + height), Scalar(255, 0, 0), 2);
            return;
        }
    }

    imshow("Sign", m_side);

    isParkingSign.data = m_parkingSign;
    pub_parkingSign.publish(isParkingSign);
}

void CCamera::findParkingArea()
{
    m_roi = Rect(0, 0, m_side.cols * 2 / 3, m_side.rows);

    cvtColor(m_side(m_roi), m_gray, CV_BGR2GRAY);
    GaussianBlur(m_gray, m_gauss, Size(3, 3), 0);
    Canny(m_gauss, m_canny, 50, 100);
    cvtColor(m_canny, m_hough, CV_GRAY2BGR);
    HoughLines(m_canny, lines, 1, PI / 180, 100);

    vector<Point2d> leftp1, leftp2, rightp1, rightp2, upperp1, upperp2, bottomp1, bottomp2;

    // average lines when detected two or more lines.
    Point2d avgLp1, avgLp2, avgRp1, avgRp2, avgUp1, avgUp2, avgBp1, avgBp2;

    // line equation in hough space : x*cos(theta) + y*sin(theta) = rho
    for (int i = 0; i < (int)lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        double c = cos(theta), s = sin(theta);

        // conversion : (rho, theta) -> (x, y)
        Point2d pt(c*rho, s*rho);
        // two points on the hough line
        Point2d delta(1000 * -s, 1000 * c);
        Point2d p1 = pt + delta;
        Point2d p2 = pt - delta;

        if (theta > min_left && theta < max_left)
        {
            // Left line
            //line(m_hough, p1, p2, Scalar(0, 0, 255));
            leftp1.push_back(p1);
            leftp2.push_back(p2);
            //cout << i << " left line : " << Point2d((int)rho, theta) << "(\BF\DE\C2\CA)" << endl;
        }

        else if (theta > min_right && theta < max_right)
        {
            // right line
            //line(m_hough, p1, p2, Scalar(255, 0, 0));
            rightp1.push_back(p1);
            rightp2.push_back(p2);
            //cout << i << " right line : " << Point2d((int)rho, theta) << "(\BF\C0\B8\A5\C2\CA)" << endl;
        }

        else if (theta > min_up && theta < max_up)
        {
            // upper
            //line(m_hough, p1, p2, Scalar(0, 255, 255));
            upperp1.push_back(p1);
            upperp2.push_back(p2);
            //cout << i << " upper line : " << Point2d((int)rho, theta) << "(\C0\A7\C2\CA)" << endl;
        }

        else if (theta > min_bottom && theta < max_bottom)
        {
            // bottom
            //line(m_hough, p1, p2, Scalar(0, 255, 0));
            bottomp1.push_back(p1);
            bottomp2.push_back(p2);
            //cout << i << " bottom line : " << Point2d((int)rho, theta) << "(\BEƷ\A1\C2\CA)" << endl;
        }
    }

    // draw	lines
    if (rightp1.size() != 0 && rightp2.size() != 0)
    {
        Point2d rt;
        size_t count = rightp1.size();

        if (count >= 2)
        {
            averageLine(rightp1, rightp2, avgRp1, avgRp2, count);
            rt = twoPoint2rhotheta(avgRp1, avgRp2);

            if (rt.y < 0) rt.y += PI;
            //if (rt.y > 0 && rt.y < 1.5707)
            //{
            //	rt.y = -rt.y;
            //}

            hough_const.push_back(rt);
            cv::line(m_hough, avgRp1, avgRp2, Scalar(255, 0, 0));
            //cout << "right line : " << rt << endl;
        }
        else
        {
            rt = twoPoint2rhotheta(rightp1[0], rightp2[0]);

            if (rt.y < 0) rt.y += PI;
            //if (rt.y > 0 && rt.y < 1.5707)
            //{
            //	rt.y = -rt.y;
            //}

            hough_const.push_back(rt);
            cv::line(m_hough, rightp1[0], rightp2[0], Scalar(255, 0, 0));
            //cout << "right line : " << rt << endl;
        }
    }

    if (leftp1.size() != 0 && leftp2.size() != 0)
    {
        Point2d rt;
        size_t count = leftp1.size();

        if (count >= 2)
        {
            averageLine(leftp1, leftp2, avgLp1, avgLp2, count);
            rt = twoPoint2rhotheta(avgLp1, avgLp2);

            if (rt.y < 0) rt.y += PI;
            //if (rt.y > 0 && rt.y < 1.5707)
            //{
            //	rt.y = -rt.y;
            //}

            hough_const.push_back(rt);
            cv::line(m_hough, avgLp1, avgLp2, Scalar(0, 0, 255));
            //cout << "left line : " << rt << endl;

        }
        else
        {
            rt = twoPoint2rhotheta(leftp1[0], leftp2[0]);

            if (rt.y < 0) rt.y += PI;
            //if (rt.y > 0 && rt.y < 1.5707)
            //{
            //	rt.y = -rt.y;
            //}

            hough_const.push_back(rt);
            cv::line(m_hough, leftp1[0], leftp2[0], Scalar(0, 0, 255));
            //cout << "left line : " << rt << endl;
        }
    }

    if (upperp1.size() != 0 && upperp2.size() != 0)
    {
        Point2d rt;
        size_t count = upperp1.size();

        if (count >= 2)
        {
            averageLine(upperp1, upperp2, avgUp1, avgUp2, count);

            rt = twoPoint2rhotheta(avgUp1, avgUp2);
            if (rt.y < 0) rt.y += PI;
            hough_const.push_back(rt);
            cv::line(m_hough, avgUp1, avgUp2, Scalar(0, 255, 255));
            //cout << "up line : " << rt << endl;
        }
        else
        {
            rt = twoPoint2rhotheta(upperp1[0], upperp2[0]);
            if (rt.y < 0) rt.y += PI;
            hough_const.push_back(rt);
            cv::line(m_hough, upperp1[0], upperp2[0], Scalar(0, 255, 255));
            //cout << "up line : " << rt << endl;
        }
    }

    if (bottomp1.size() != 0 && bottomp2.size() != 0)
    {
        Point2d rt;
        size_t count = bottomp1.size();
        if (count >= 2)
        {
            averageLine(bottomp1, bottomp2, avgBp1, avgBp2, count);

            rt = twoPoint2rhotheta(avgBp1, avgBp2);
            if (rt.y < 0) rt.y += PI;
            hough_const.push_back(rt);
            cv::line(m_hough, avgBp1, avgBp2, Scalar(0, 255, 0));
            //cout << "down line : " << rt << endl;
        }
        else
        {
            rt = twoPoint2rhotheta(bottomp1[0], bottomp2[0]);
            if (rt.y < 0) rt.y += PI;
            hough_const.push_back(rt);
            cv::line(m_hough, bottomp1[0], bottomp2[0], Scalar(0, 255, 0));
            //cout << "down line : " << rt << endl;
        }
    }

    int cross_count = 0;
    if (hough_const.size() > 0 && hough_const.size() <= 4)
    {
        for (int i = 0; i < hough_const.size() - 1; i++)
        {
            for (int j = i + 1; j < hough_const.size(); j++)
            {
                if (hough_const.size() == 4)
                {
                    if((i == 0 && j == 1) || (i == 2 && j == 3)) continue;
                }

                Point2d tcp = calc_crosspoint(hough_const[i], hough_const[j]);

                if (tcp.x > 0 && tcp.x < m_canny.cols && tcp.y > 30 && tcp.y < m_canny.rows)
                {
                    cp.push_back(tcp);
                    cross_count++;
                    //cout << cross_count << "crosspoint count : " << tcp << endl;
                }
            }
        }

        for (int i = 0; i < cp.size(); i++)
        {
            circle(m_hough, cp[i], 2, Scalar(0, 0, 255), 2);
        }
    }

    imshow("Parking", m_hough);

    Size warpSize(200, 200);
    Mat warpImg(warpSize, m_canny.type());

    if (cp.size() == 4)
    {
        vector<Point2f> corners(4);
        for (int i = 0; i < 4; i++)
        {
            corners[i] = cp[i];
        }

        //Warping Point
        vector<Point2f> warpCorners(4);
        warpCorners[0] = Point2f(0, 0);
        warpCorners[1] = Point2f(warpImg.cols, 0);
        warpCorners[2] = Point2f(0, warpImg.rows);
        warpCorners[3] = Point2f(warpImg.cols, warpImg.rows);

        //Transformation Matrix
        Mat trans = getPerspectiveTransform(corners, warpCorners);

        //Warping
        warpPerspective(m_canny, warpImg, trans, warpSize);

        int edge_count = 0;
        int row = warpImg.rows;
        int col = warpImg.cols;

        for (int r = row * 0.5; r <= row * 0.8; r++)
        {
            for (int c = col * 0.2; c <= col * 0.8; c++)
            {
                if (warpImg.at<uchar>(r, c) == 255)
                {
                    edge_count++;
                }
            }
        }

        cout << edge_count << endl;

        if (edge_count > 0)
        {
            cout << "주차 불가" << endl;
        }

        else
        {
            // transfer/publish bool data
            m_parkingArea = true;
        }
    }

    else
    {
        //warpImg.zeros(warpSize, CV_8U);
        warpImg.setTo(Scalar(0));
    }

    isParkingArea.data = m_parkingArea;
    pub_parkingArea.publish(isParkingArea);

    imshow("check_Area", warpImg);

    if (!lines.empty()) lines.clear();
    if (!hough_const.empty()) hough_const.clear();
    if (!cp.empty()) cp.clear();
}

// Find (rho, theta) of hough line using two points which is on the line.
Point2d twoPoint2rhotheta(Point2d p1, Point2d p2)
{
    Point2d rt;
    // y = ax + b
    double a = (p2.y - p1.y) / (p2.x - p1.x);
    double b = p1.y - a * p1.x;
    double slope = -1 / a;

    // cross y = ax + b & y = -1/a x : right_cross
    Point2d cross;
    cross.x = -(a*b) / (a*a + 1);
    cross.y = b / (a*a + 1);

    // convert xy to rhotheta
    rt.x = sqrt((cross.x * cross.x) + (cross.y * cross.y));
    rt.y = atan(cross.y / cross.x);

    return rt;
}

// Calculate crosspoint location at hough space
Point2d calc_crosspoint(Point2d rt1, Point2d rt2)
{
    double xden, xnum, yden, ynum;
    double x, y;

    xnum = rt2.x / sin(rt2.y) - rt1.x / sin(rt1.y);
    xden = -cos(rt1.y) / sin(rt1.y) + cos(rt2.y) / sin(rt2.y);
    x = xnum / xden;

    ynum = rt1.x - x * cos(rt1.y);
    yden = sin(rt1.y);
    y = ynum / yden;

    return Point2d(x, y);
}

// choose one line which is nearest parking line
void averageLine(vector<Point2d> p1, vector<Point2d> p2, Point2d& avgp1, Point2d& avgp2, int count)
{
    double sumx1 = 0.0, sumy1 = 0.0;
    double sumx2 = 0.0, sumy2 = 0.0;
    // each of average : start point & end point
    for (int j = 0; j < count; j++)
    {
        sumx1 += p1[j].x;
        sumy1 += p1[j].y;
    }
    avgp1 = Point2d(sumx1 / count, sumy1 / count);
    for (int j = 0; j < count; j++)
    {
        sumx2 += p2[j].x;
        sumy2 += p2[j].y;
    }
    avgp2 = Point2d(sumx2 / count, sumy2 / count);
}
