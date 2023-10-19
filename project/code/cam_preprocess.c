/**
* @filename: cam_preprocess[摄像头图像预处理]
* @filepath: /MineSrc/app/CamAlgorithm
* @function: 摄像头图像的预处理，包括压缩、二值化等
* @author:   CyberLab-AI_Vision
*/
#include "cam_preprocess.h"

//二值化宏定义
#define CAM_POINT_GRAY(x, y)             (CmpressedImg[y][x])   //获取灰度值
#define CAM_POINT_BINARY_SET(x, y)       (BinaryImg[y][x] = 1)  //二值化变白点
#define CAM_POINT_BINARY_RESET(x, y)     (BinaryImg[y][x] = 0)     //二值化变黑点

//连通域输入图像宏定义
#define CDM_POINT_WHITE(x, y)            (BinaryImg[y][x] == 1)
#define CDM_POINT_BLACK(x, y)            (BinaryImg[y][x] == 0)


#define CDM_offset  (0) //用于控制连通区域的最小连接点数(当连通区域在上一行边界时)，取 offset2 > 0
#define RUN_MIN_LEN (0) //去掉长度小于等于0的点，（0）实际长度是1个像素


//图像参数配置结构体变量
Img_Config ImgConf =
        {
                .Threshold = 0,
                .minThres = 55,
                .maxThres = 180,
                .white_threshold = 0.5f,
        };


//图像数据定义
uint8 CmpressedImg[IMG_ROW][IMG_COL];            //压缩图像
uint8 BinaryImg[IMG_ROW][IMG_COL];                        //原始二值化图像
uint8 BinaryImg_CDM[IMG_ROW][IMG_COL];                //连通域后输出的二值化图像
uint8 JumpRow[IMG_ROW], JumpColumn[IMG_COL];        //采样点信息


ConDomainNode cdm;  //连通域结构体变量

//采样点初始化
void JumpInit(void) {
    float jump_c jump_column = (float) MT9V03X_W / (float) IMG_COL;
    jolumn = 0;
    float jump_row = 0;
    uint8_t count = 0;

   ump_row = (float) MT9V03X_H / (float) IMG_ROW;

    for (count = 0; count < IMG_COL; ++count) {
        JumpColumn[count] = (int) (jump_column * count);
    }
    for (count = 0; count < IMG_ROW; ++count) {
        JumpRow[count] = (int) (jump_row * count);
    }
}

//图像压缩
void ImageCompression() {
    uint8_t row, column = 0;

    for (row = 0; row < IMG_ROW; ++row) {
        for (column = 0; column < IMG_COL; ++column) {
            CmpressedImg[row][column] = mt9v03x_image[JumpRow[row]][JumpColumn[column]];
        }
    }
}


/**
 * @func	  	自动阈值函数
 * @Author		Wei Zhou
 * @param 		{type} Image {uint8_t *}  需要阈值的原始图像
 *               row                图像的行数
 *               column             图像的列数
 *               thres              向当前的阈值
 *               deltThres          最小两次阈值差值
 *               low                输出的最小阈值
 *               top                输出的最大阈值
 * @return: 	阈值
 */
uint8_t Automatic_Threshold(const uint8_t thres, const uint8_t deltThres, const int16_t low, const int16_t top) {
    uint8_t lastThres;                  // 阈值
    uint8_t newThres = thres;           // 更新后的阈值
    uint32_t countValue_LessThres;       // 保存灰度小于阈值的点的总灰度
    uint32_t countValue_MoreThres;       // 保存灰度大于阈值的点的总灰度
    uint16_t countNum_LessThres;         // 保存灰度小于阈值的点的个数
    uint16_t countNum_MoreThres;         // 保存灰度大于阈值的点的个数
    uint32_t averageValue_LessThres;
    uint32_t averageValue_MoreThres;
    static uint32 lastaverageValue_LessThreshold = 0;
    static uint32 lastaverageValue_MoreThreshold = 0;
    uint8_t r, c;

    uint8_t times = 0;

    do {
        if (times++ > 2) break;

        countValue_LessThres = 0;
        countValue_MoreThres = 0;
        countNum_LessThres = 0;
        countNum_MoreThres = 0;

        lastThres = newThres;

        for (r = IMG_TOP; r < IMG_ROW; r++) {
            for (c = 0; c < IMG_COL; c++) {
                if (CAM_POINT_GRAY(c, r) < newThres) {
                    countValue_LessThres += CAM_POINT_GRAY(c, r);
                    countNum_LessThres++;
                } else {
                    countValue_MoreThres += CAM_POINT_GRAY(c, r);
                    countNum_MoreThres++;
                }
            }
        }

//				if (countNum_LessThres < 100) // 小于 100 个黑点，阈值不变
//        {
//        	return thres;
//        }

        if (countNum_LessThres == 0) {
            averageValue_LessThres = lastaverageValue_LessThreshold;
        } else {
            averageValue_LessThres = countValue_LessThres / countNum_LessThres;
            lastaverageValue_LessThreshold = averageValue_LessThres;
        }

        if (countNum_MoreThres == 0) {
            averageValue_MoreThres = lastaverageValue_MoreThreshold;
        } else {
            averageValue_MoreThres = countValue_MoreThres / countNum_MoreThres;
            lastaverageValue_MoreThreshold = averageValue_MoreThres;
        }

        newThres = (uint8_t) ((averageValue_LessThres * ImgConf.white_threshold +
                               averageValue_MoreThres * (1.0f - ImgConf.white_threshold)));
    } while (_abs(newThres - lastThres) > deltThres);

    return _limit(newThres, low, top);
}

static uint8
Automatic_Threshold_new(uint8 *ImageCmprsData, uint8 row, uint8 column, uint8 *threshold, uint8 deltThreshold) {
    uint8 lastThreshold;                  // 阈值
    uint8 newThreshold = *threshold;      // 更新后的阈值
    uint32 countValue_LessThreshold;       // 保存灰度小于阈值的点的总灰度
    uint32 countValue_MoreThreshold;       // 保存灰度大于阈值的点的总灰度
    uint16 countNum_LessThreshold;         // 保存灰度小于阈值的点的个数
    uint16 countNum_MoreThreshold;         // 保存灰度大于阈值的点的个数
    uint32 averageValue_LessThreshold;
    uint32 averageValue_MoreThreshold;
    static uint32 lastaverageValue_LessThreshold = 0;
    static uint32 lastaverageValue_MoreThreshold = 0;
    uint8 r, c;

    do {
        countValue_LessThreshold = 0;
        countValue_MoreThreshold = 0;
        countNum_LessThreshold = 0;
        countNum_MoreThreshold = 0;

        lastThreshold = newThreshold;

        for (r = 0; r < row; r++) {
            for (c = 0; c < column; c++) {
                if (*(ImageCmprsData + r * column + c) < newThreshold) {
                    countValue_LessThreshold += *(ImageCmprsData + r * column + c);
                    countNum_LessThreshold++;
                } else {
                    countValue_MoreThreshold += *(ImageCmprsData + r * column + c);
                    countNum_MoreThreshold++;
                }
            }
        }

        if (countNum_LessThreshold == 0) {
            averageValue_LessThreshold = lastaverageValue_LessThreshold;
        } else {
            averageValue_LessThreshold = countValue_LessThreshold / countNum_LessThreshold;
            lastaverageValue_LessThreshold = averageValue_LessThreshold;
        }

        if (countNum_MoreThreshold == 0) {
            averageValue_MoreThreshold = lastaverageValue_MoreThreshold;
        } else {
            averageValue_MoreThreshold = countValue_MoreThreshold / countNum_MoreThreshold;
            lastaverageValue_MoreThreshold = averageValue_MoreThreshold;
        }

        newThreshold = (int) (averageValue_LessThreshold * ImgConf.white_threshold +
                              averageValue_MoreThreshold * (1.0f - ImgConf.white_threshold));
    } while (_abs(newThreshold - lastThreshold) > deltThreshold);//设置迭代条件
    //while(0);//此处仅对每帧图像进行一次迭代

    return *threshold = _limit(newThreshold, ImgConf.minThres, ImgConf.maxThres);
}

//图像二值化函数
void GetBinarizedImage(uint8_t Threshold) {
    int16_t r, c;

    for (r = 0; r < IMG_ROW; ++r) {
        for (c = 0; c < IMG_COL; ++c) {
            if (CAM_POINT_GRAY(c, r) > Threshold) {
                CAM_POINT_BINARY_SET(c, r);
            } else {
                CAM_POINT_BINARY_RESET(c, r);
            }
        }
    }
}



/***********************连通域相关函数*******************************/
/***********************连通域相关函数*******************************/
/***********************连通域相关函数*******************************/
/*👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇👇*/

/**
 * @func		在一副图像中寻找连续区域 （一行中的一条白点）
 * @Author	Wei Zhou
 * @param		{ConDomainNode *}cdm   保存连通域算法信息的结构体对象指针
 * @return	0: 寻找完成； -1：超出数组表示范围
 */
static int16_t FillRunVectors(ConDomainNode *cdm) {
    int16_t anyRun = 0; // 标记是否有在一行中找到连通域
    int16_t pstRun = 0;
    int16_t penRun = 0;
    int16_t prowRun = 0;

    cdm->numOfRuns = 0;

    // 从最后一行开始
    for (int16_t r = IMG_ROW - 1; r >= IMG_TOP; r--) {
        anyRun = 0;

        if (CDM_POINT_WHITE(0, r)) // 如果第一个点是白点， 记录当前区域
        {
            cdm->numOfRuns++;
            cdm->stRun[pstRun++] = 0; // 区域起始点为0
            cdm->rowRun[prowRun++] = r;
            anyRun = 1;
        }

        for (int16_t c = 1; c < IMG_COL; ++c) // 从第二列开始
        {
            if (CDM_POINT_BLACK(c - 1, r) && CDM_POINT_WHITE(c, r)) // 左黑，当白，记录新区域
            {
                cdm->numOfRuns++;
                cdm->stRun[pstRun++] = c;
                cdm->rowRun[prowRun++] = r;
                anyRun = 1;

            } else if (CDM_POINT_WHITE(c - 1, r) && CDM_POINT_BLACK(c, r)) // 左白，当黑，记录上一区域终点
            {
                cdm->enRun[penRun++] = c - 1;
            }
            if (cdm->numOfRuns >= MAX_AREA) // 控制区域数量
            {
                return 0; // 超出表示范围
            }
        }
        if (CDM_POINT_WHITE(IMG_COL - 1, r)) // 如果最后一个点是白，强制记录当前点为区域终点
        {
            cdm->enRun[penRun++] = IMG_COL - 1;
        }
        if (anyRun == 0) // 如果这行没有任何区域，停止向上一行搜索
        {
            break;
        }
    }
    return 1; // 标记和记录完成
}

/**
 * @func		标记连通区域
 * @Author	Wei Zhou
 * @param 	{ConDomainNode *} cdm   保存连通域算法信息的结构体对象指针
 */
static void LabeledConDomain(ConDomainNode *cdm) {
    int16_t curRowIdx = IMG_ROW;
    int16_t firstRunOnCur = 0;
    int16_t firstRunOnPre = 0;
    int16_t lastRunOnPre = -1;

    cdm->peq = 0;

    for (int16_t i = 0; i < cdm->numOfRuns; ++i) {
        // 如果是该行的第一个run，则更新上一行 [第一个run] 和 [最后一个run] 的序号
        if (cdm->rowRun[i] != curRowIdx) {
            curRowIdx = cdm->rowRun[i];   // 更新行的序号
            firstRunOnPre = firstRunOnCur;
            lastRunOnPre = i - 1;
            firstRunOnCur = i;
        }

        // int16_t offset = (CDM_offset * (IMG_ROW-1 - cdm->rowRun[i]) / (IMG_ROW-1 - IMG_TOP));

        if (cdm->enRun[i] - cdm->stRun[i] <= RUN_MIN_LEN) { continue; } // 去掉长度为0的点

        // 遍历上一行的所有run，判断是否于当前run有重合的区域
        for (int16_t j = firstRunOnPre; j <= lastRunOnPre; j++) // j 是上一行的区域（依次从 BOTTOM 到 TOP）
        {
            // 区域重合 且 处于相邻的两行
            if (cdm->stRun[i] < IMG_COL - CDM_offset && cdm->enRun[i] > CDM_offset) {
                if (cdm->stRun[i] <= cdm->enRun[j] - CDM_offset && cdm->enRun[i] >= cdm->stRun[j] + CDM_offset) {
                    cdm->eq_l[cdm->peq] = j;      // 保存等价对
                    cdm->eq_r[cdm->peq++] = i;
                }
            } else if (cdm->stRun[i] >= IMG_COL - CDM_offset) // 考虑右边界情况
            {
                if (cdm->stRun[i] <= cdm->enRun[j]) {
                    cdm->eq_l[cdm->peq] = j;      // 保存等价对
                    cdm->eq_r[cdm->peq++] = i;
                }
            } else if (cdm->enRun[i] <= CDM_offset)   // 考虑左边界情况
            {
                if (cdm->enRun[i] >= cdm->stRun[j]) {
                    cdm->eq_l[cdm->peq] = j;      // 保存等价对
                    cdm->eq_r[cdm->peq++] = i;
                }
            }
        }
    }
}

//连通域算法控制递归的次数
#define MAX_NEXT_TIMES (MAX_AREA-1)
#define MAX_PREV_TIMES (MAX_AREA-1)

static int16_t next_times;
static int16_t prev_times;

/**
 * @func	 	向后寻找等价区域
 * @Author 	Wei Zhou
 * @param 	{ConDomainNode *} cdm     保存连通域算法信息的结构体对象指针
 *             {int16_t}  	begin   	开始寻找标号
 *             {int16_t}			cur     等价标号
 */
static void addEquival_next(ConDomainNode *cdm, int16_t begin, int16_t cur) {
    int16_t i;

    if (--next_times <= 0) { return; }
    for (i = begin; i < cdm->peq; ++i) {
        if (cdm->eq_l[i] == cur) {
            cdm->equival[cdm->eq_r[i]] = 1; // 添加到等价表中
            if (cdm->eq_r[i] > cdm->lastSign_equival) { cdm->lastSign_equival = cdm->eq_r[i]; }
            addEquival_next(cdm, i + 1, cdm->eq_r[i]); // 寻找下一个
        }
        if (cdm->eq_l[i] > cur) {
            break;
        }
    }
}

/**
 * @func		向前寻找等价区域
 * @Author	Wei Zhou
 * @param		{ConDomainNode *} 	cdm     保存连通域算法信息的结构体对象指针
 *              {int16_t} 			begin   开始寻找标号
 *              {int16_t}				cur     等价标号
 */
static void addEquival_prev(ConDomainNode *cdm, int16_t begin, int16_t cur) {
    int16_t i;

    if (--prev_times <= 0) { ; }
    for (i = begin; i >= 0; --i) {
        if (cdm->eq_r[i] == cur) {
            cdm->equival[cdm->eq_l[i]] = 1; // 添加到等价表中
            addEquival_prev(cdm, i - 1, cdm->eq_l[i]); // 寻找下一个
        }
        if (cdm->eq_r[i] < cur) {
            break;
        }
    }
}

/**
 * @func		连通域
 * @Author	Wei Zhou
 * @param 	{ConDomainNode *} 	cdm   保存连通域算法信息的结构体对象指针
 * @BUG			2020/7/8
 *      		memset(cdm->equival, 0, sizeof(int16_t) * MAX_AREA); 初始化顺序上移   ？？？
 *      		修改 cdm->equival[mainRun] = 1 改为 cdm->equival[0] = mainRun
 */
void Connected_Domain(ConDomainNode *cdm, int16_t find_back) {
    int16_t mainRun = 0; // 主要区域

    memset(CDM_TEMP_BINARY, 0, sizeof(char) * IMG_ROW * IMG_COL);

    if (FillRunVectors(cdm))    // 在一副图像中寻找连续区域 （一行中的一条白点）
    {
        LabeledConDomain(cdm); // 标记连通区域

        // 寻找主要区域
        for (int16_t i = 1; i < cdm->peq; ++i) {
            if (cdm->rowRun[i] == IMG_ROW - 1) {
                if (cdm->enRun[i] - cdm->stRun[i] > cdm->enRun[mainRun] - cdm->stRun[mainRun]) {
                    mainRun = i;
                }
            } else {
                break;
            }
        }

        memset(cdm->equival, 0, sizeof(int16_t) * MAX_AREA);

        cdm->lastSign_equival = 0;
        cdm->equival[mainRun] = 1; // 2020/7/8 修正BUG

        next_times = MAX_NEXT_TIMES;
        prev_times = MAX_PREV_TIMES;

        addEquival_next(cdm, 0, mainRun); // 从 mainRun 开始，向上寻找
        if (find_back) {
            addEquival_prev(cdm, cdm->peq - 1, cdm->lastSign_equival); // 从cdm->last_equival开始向下寻找
        }
    }
}

/**
 * @brief		图像分割
 * @Author	Wei Zhou
 * @param		{ConDomainNode *} cdm   保存连通域算法信息的结构体对象指针
 *          {ELEMENT_SIGN} 		ele   元素类型
 */
void CDM_Segment(ConDomainNode *cdm) {
    for (int16_t i = 0; i <= cdm->lastSign_equival; ++i) {
        if (cdm->equival[i]) {
            // 图像分割二值化
            for (int16_t c = cdm->stRun[i]; c <= cdm->enRun[i]; ++c) {
                CDM_TEMP_POINT_SET(c, cdm->rowRun[i]);
            }
        }
    }
}

/*👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆👆*/
/***********************连通域相关函数*******************************/
/***********************连通域相关函数*******************************/
/***********************连通域相关函数*******************************/

/**
 * @description: 主道图像分割（将左边割掉）
 * @Author: Wei Zhou
 * @param {type}
 */
int16 CDM_MainStreet_Segment_LEFT_UP(ConDomainNode *cdm, int16 LEFT, int16 TOP, _POINT_ *bot, _POINT_ *top) {
    int16 i, r, c, stopc;
    int16 segment_Run = 0;
    float k, b;

    if (top->x > bot->x && top->y < bot->y) {
        if (TOP < top->y) {
            for (i = 0; i <= cdm->lastSign_equival; i++) {
                if (cdm->equival[i]) {
                    if (cdm->stRun[i] < top->x && cdm->rowRun[i] < top->y) {
                        for (c = cdm->stRun[i]; c <= cdm->enRun[i]; c++) {
                            CDM_TEMP_POINT_RESET(c, cdm->rowRun[i]);
                        }
                        segment_Run++;
                    }
                }
            }
        }

        k = (float) (top->y - bot->y) / (float) (top->x - bot->x); // 斜率
        b = top->y - k * top->x;

        for (r = top->y; r <= bot->y; r++) {
            stopc = (r - b) / k + 0.5f;
            for (c = LEFT; c <= stopc; c++) {
                if (c < 0 || c > IMG_COL - 1) { continue; }
                if (r < 0 || r > IMG_ROW - 1) { continue; }
                CDM_TEMP_POINT_RESET(c, r);
            }
        }
        return segment_Run;
    }
    return 0;
}

/**
 * @description: 主道图像分割
 * @Author: Wei Zhou
 * @param {type}
 */
int16 CDM_MainStreet_Segment_RIGHT_UP(ConDomainNode *cdm, int16 RIGHT, int16 TOP, _POINT_ *bot, _POINT_ *top) {
    int16 i, r, c, startc;
    int16 segment_Run = 0;
    float k, b;

    if (top->x < bot->x && top->y < bot->y) {
        if (TOP < top->y) {
            for (i = 0; i <= cdm->lastSign_equival; i++) {
                if (cdm->equival[i]) {
                    if (cdm->enRun[i] > top->x && cdm->rowRun[i] < top->y) {
                        for (c = cdm->stRun[i]; c <= cdm->enRun[i]; c++) {
                            CDM_TEMP_POINT_RESET(c, cdm->rowRun[i]);
                        }
                        segment_Run++;
                    }
                }
            }
        }

        k = (float) (top->y - bot->y) / (float) (top->x - bot->x); // 斜率
        b = top->y - k * top->x;

        for (r = top->y; r <= bot->y; r++) {
            startc = (r - b) / k + 0.5f;

            for (c = startc; c <= RIGHT; c++) {
                if (c < 0 || c > IMG_COL - 1) { return True; }
                if (r < 0 || r > IMG_ROW - 1) { return True; }
                CDM_TEMP_POINT_RESET(c, r);
            }
        }
        return segment_Run;
    }
    return 0;
}

/**
 * @description: 找主道图像分割点
 * @Author: Wei Zhou
 * @param {type} cdm    {ConDomainNode *} 保存连通域算法信息的结构体对象指针
 *               ele    {ELEMENT_SIGN} 元素类型
 */
#define MAIN_STREET_MIN_LEN     (8)

int16 CDM_MainStreet_Point(ConDomainNode *cdm, MainStreetType mst, _POINT_ *point) {
    int16 i;
    int16 find_sign;
    _POINT_ temp;

    point->x = 0;
    point->y = 0;

    find_sign = 0;
    temp.y = 0;

    for (i = cdm->lastSign_equival; i > 0; --i) {
        if (cdm->equival[i] && cdm->equival[i - 1]) {
            if (cdm->rowRun[i] == cdm->rowRun[i - 1]) {
                if (cdm->enRun[i] - cdm->stRun[i] > MAIN_STREET_MIN_LEN &&
                    cdm->enRun[i - 1] - cdm->stRun[i - 1] > MAIN_STREET_MIN_LEN) {
                    if (mst == MainStreet_RIGHT) {
                        temp.x = cdm->enRun[i - 1];
                        temp.y = cdm->rowRun[i - 1];
                        find_sign = i - 1;
                    } else if (mst == MainStreet_LEFT) {
                        temp.x = cdm->stRun[i];
                        temp.y = cdm->rowRun[i];
                        find_sign = i;
                    }

                    // 如果上方为白色或者下方为黑色，表明不是寻找的分割点
                    if (CDM_TEMP_POINT_WHITE(temp.x, temp.y - 1) || CDM_TEMP_POINT_BLACK(temp.x, temp.y + 1)) {
                        find_sign = 0;
                        continue;
                    }
                }
            }
        }
    }

    if (find_sign > 0) {
        for (i = find_sign; i > 0; --i) {
            if (cdm->rowRun[i] - temp.y > 1) {
                break;
            }
            if (cdm->enRun[i] - temp.x<10 && cdm->enRun[i] - cdm->stRun[i]>MAIN_STREET_MIN_LEN) {
                temp.x = cdm->enRun[i];
                temp.y = cdm->rowRun[i];
            }
        }

//        if(temp.y >= point->y && temp.y - point->y < 30)
//        {
        point->y = temp.y;
        point->x = temp.x;
        return True;
//        }
    }

    return False;
}

//图像预处理(整合函数)
void ImgPreprocess(void) {
    ImageCompression();    //图像压缩
    ImgConf.Threshold = Automatic_Threshold(ImgConf.Threshold, 1, ImgConf.minThres, ImgConf.maxThres);//获取阈值
    GetBinarizedImage(ImgConf.Threshold);        //图像二值化
    Connected_Domain(&cdm, 1);        //连通域算法
    CDM_Segment(&cdm);        //主道分割

}


