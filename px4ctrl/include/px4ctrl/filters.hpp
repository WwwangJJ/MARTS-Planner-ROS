#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <ros/ros.h>
#include <iostream>
#include <cmath>


namespace filters
{

const int max_numStages = 100;

class biquad_cascade_df1_params_t
{
public:    
    uint32_t numStages;      /**< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
    double scale;
    double pCoeffs[100];      /**< Points to the array of coefficients.  The array is of length 5*numStages. */
    double pState[100];       /**< Points to the array of state coefficients.  The array is of length 4*numStages. */

    void param_333_58_order2(void)
    {
        numStages = 1;
        scale = 0.166238273180484175117754830353078432381;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 0.563274599637472728197451488085789605975, -0.228227692359409456424046425127016846091
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_54_order2(void)
    {
        numStages = 1;
        scale = 0.148457080118556378200977974302077200264;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 0.654516041228168199417325467948103323579, -0.248344361702393850999115443300979677588
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_50_order2(void)
    {
        numStages = 1;
        scale = 0.131316543269871938148796175482857506722;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 0.746631727846268300474719126214040443301, -0.271897900925756108581055059403297491372
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_30_order2(void)
    {
        numStages = 1;
        scale = 0.056544637586181112354211109050083905458 ;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.223911541924341372933326965721789747477, -0.450090092269065933372473864437779411674
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }


    void param_333_24_order2(void)
    {
        numStages = 1;
        scale = 0.038488877353061466679573499050093232654;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.373490222473645161471722531132400035858, -0.527445731885891055945592142961686477065
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_20_order2(void)
    {
        numStages = 1;
        scale = 0.027909411748536647884799677399314532522;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.474969547350690524112337698170449584723, -0.586607194344836990751446137437596917152
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_16_order2(void)
    {
        numStages = 1;
        scale = 0.018684321580149008956706069284336990677;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.577822109095380564980359849869273602962, -0.652559395415976850607364667666843160987
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_12_order2(void)
    {
        numStages = 1;
        scale = 0.011013663590770413416852768762055347906;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.681958981271259645495774748269468545914, -0.726013635634341292224291919410461559892
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_9_order2(void)
    {
        numStages = 1;
        scale = 0.006424327270838326234270176939844532171;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.760811931649823414147704170318320393562, -0.786509240733176961946071514830691739917
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_8_order2(void)
    {
        numStages = 1;
        scale = 0.005139032580981528486319120929692871869;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.787221016261407502412339454167522490025, -0.807777146585333616357615937886293977499
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_7_order2(void)
    {
        numStages = 1;
        scale = 0.003983887172802479115230056549989967607;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.813685339069859070448842430778313428164 , -0.829620887761068903643035810091532766819
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_6_order2(void)
    {
        numStages = 1;
        scale = 0.002963971055125952120540278045268678397;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.840199942341331995621089845371898263693, -0.852055826561835760735164058132795616984
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_5_order2(void)
    {
        numStages = 1;
        scale = 0.00208460162050542107889383736107902223;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.866759379673801122478948855132330209017, -0.875097786155823009757170893863076344132
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_4_order2(void)
    {
        numStages = 1;
        scale = 0.00135134386463272161520154934777337985;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.893357683658256851444434687437023967505, -0.89876305911678766591421663179062306881
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_333_3_order2(void)
    {
        numStages = 1;
        scale = 0.00077002098653899779534537373137936811;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.91998833154299419767596646124729886651, -0.923068415489149862729334472533082589507
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_200_12_order2(void)
    {
        numStages = 1;
        scale = 0.027859766117136031127188289246987551451;

        //12hz
        static double set_coeff[5] = 
        {
            1, 2, 1, 1.475480443592646073724949928873684257269, -0.586919508061190309256005548377288505435
        };

        memcpy(pCoeffs, set_coeff, 5 * sizeof(double));
    }

    void param_200_10(void)
    {
        numStages = 2;
        scale = 0.002141880667055908425699950470288968063 * 0.002041518402531028453239514419692568481;

        //10hz
        static double set_coeff[10] = 
        {
            1, 2, 1, 1.921908893578823107617381538148038089275, -0.930476416247046578256174598209327086806,
            1, 2, 1, 1.831853863092363976505794198601506650448, -0.840019936702487868274147331248968839645
        };

        memcpy(pCoeffs, set_coeff, 10 * sizeof(double));
    }

    void param_200_20(void)
    {
        numStages = 2;
        scale = 0.077956340516462552248100337237701751292 * 0.061885195299764481080551092873065499589;

        // 20hz
        static double set_coeff[10] = 
        {
            1, 2, 1, 1.320913430819426359619228605879470705986, -0.632738792885276568611629954830277711153,
            1, 2, 1, 1.048599576362611696112026038463227450848, -0.29614035756166950941192794743983540684
        };

        memcpy(pCoeffs, set_coeff, 10 * sizeof(double));
    }

    void param_200_40(void)
    {
        numStages = 2;
        scale = 0.25330151252771304637434468531864695251 * 0.183902994386372503621274177021405193955;

        //40hz
        static double set_coeff[10] = 
        {
            1, 2, 1, 0.453119520652384921710620346857467666268, -0.466325570763236940674545394358574412763,
            1, 2, 1, 0.32897567737095306794614657519559841603, -0.064587654916443026920092052023392170668
        };

        memcpy(pCoeffs, set_coeff, 10 * sizeof(double));
    }


};


class biquad_cascade_df1
{
public:
    biquad_cascade_df1_params_t param;

    // void init(biquad_cascade_df1_params_t &set_param)
    // {
    //     param = set_param;
    //     /* Clear state buffer and size is always 4 * numStages */
    //     memset(param.pState, 0, (4 * (uint32_t) param.numStages) * sizeof(double));
    // }

    // biquad_cascade_df1(biquad_cascade_df1_params_t &set_param)
    // {
    //     init(set_param);
    // }

    biquad_cascade_df1(){}

    void step_with_scale(double *pSrc, double *pDst, uint32_t blockSize)
    {   
        step(pSrc, pDst, blockSize);
        for(int i = 0; i <= blockSize - 1; i++)
        {
            pDst[i] *= param.scale;
        }
    }

/**    
 * @param[in]  *pSrc      points to the block of input data.    
 * @param[out] *pDst      points to the block of output data.    
 * @param[in]  blockSize  number of samples to process per call.    
 * @return     none.    
 *    
 */

    void step(double *pSrc, double *pDst, uint32_t blockSize)
    {
        double *pIn = pSrc;                          /*  source pointer            */
        double *pOut = pDst;                         /*  destination pointer       */
        double *temp_pState = param.pState;          /*  pState pointer            */
        double *temp_pCoeffs = param.pCoeffs;        /*  coefficient pointer       */
        double acc;                                  /*  Simulates the accumulator */
        double b0, b1, b2, a1, a2;                   /*  Filter coefficients       */
        double Xn1, Xn2, Yn1, Yn2;                   /*  Filter pState variables   */
        double Xn;                                   /*  temporary input           */
        uint32_t sample, stage = param.numStages;    /*  loop counters             */

        do
        {
            /* Reading the coefficients */
            b0 = *temp_pCoeffs++;
            b1 = *temp_pCoeffs++;
            b2 = *temp_pCoeffs++;
            a1 = *temp_pCoeffs++;
            a2 = *temp_pCoeffs++;

            /* Reading the pState values */
            Xn1 = temp_pState[0];
            Xn2 = temp_pState[1];
            Yn1 = temp_pState[2];
            Yn2 = temp_pState[3];

            /* Apply loop unrolling and compute 4 output values simultaneously. */
            /*      The variable acc hold output values that are being computed:    
            *    
            *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
            *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
            *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
            *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1]   + a2 * y[n-2]    
            */

            sample = blockSize >> 2u;

            /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
            ** a second loop below computes the remaining 1 to 3 samples. */
            while(sample > 0u)
            {
            /* Read the first input */
            Xn = *pIn++;

            /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
            Yn2 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

            /* Store the result in the accumulator in the destination buffer. */
            *pOut++ = Yn2;

            /* Every time after the output is computed state should be updated. */
            /* The states should be updated as:  */
            /* Xn2 = Xn1    */
            /* Xn1 = Xn     */
            /* Yn2 = Yn1    */
            /* Yn1 = acc   */

            /* Read the second input */
            Xn2 = *pIn++;

            /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
            Yn1 = (b0 * Xn2) + (b1 * Xn) + (b2 * Xn1) + (a1 * Yn2) + (a2 * Yn1);

            /* Store the result in the accumulator in the destination buffer. */
            *pOut++ = Yn1;

            /* Every time after the output is computed state should be updated. */
            /* The states should be updated as:  */
            /* Xn2 = Xn1    */
            /* Xn1 = Xn     */
            /* Yn2 = Yn1    */
            /* Yn1 = acc   */

            /* Read the third input */
            Xn1 = *pIn++;

            /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
            Yn2 = (b0 * Xn1) + (b1 * Xn2) + (b2 * Xn) + (a1 * Yn1) + (a2 * Yn2);

            /* Store the result in the accumulator in the destination buffer. */
            *pOut++ = Yn2;

            /* Every time after the output is computed state should be updated. */
            /* The states should be updated as: */
            /* Xn2 = Xn1    */
            /* Xn1 = Xn     */
            /* Yn2 = Yn1    */
            /* Yn1 = acc   */

            /* Read the forth input */
            Xn = *pIn++;

            /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
            Yn1 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn2) + (a2 * Yn1);

            /* Store the result in the accumulator in the destination buffer. */
            *pOut++ = Yn1;

            /* Every time after the output is computed state should be updated. */
            /* The states should be updated as:  */
            /* Xn2 = Xn1    */
            /* Xn1 = Xn     */
            /* Yn2 = Yn1    */
            /* Yn1 = acc   */
            Xn2 = Xn1;
            Xn1 = Xn;

            /* decrement the loop counter */
            sample--;

            }

            /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
            ** No loop unrolling is used. */
            sample = blockSize & 0x3u;

            while(sample > 0u)
            {
            /* Read the input */
            Xn = *pIn++;

            /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
            acc = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

            /* Store the result in the accumulator in the destination buffer. */
            *pOut++ = acc;

            /* Every time after the output is computed state should be updated. */
            /* The states should be updated as:    */
            /* Xn2 = Xn1    */
            /* Xn1 = Xn     */
            /* Yn2 = Yn1    */
            /* Yn1 = acc   */
            Xn2 = Xn1;
            Xn1 = Xn;
            Yn2 = Yn1;
            Yn1 = acc;

            /* decrement the loop counter */
            sample--;

            }

            /*  Store the updated state variables back into the pState array */
            *temp_pState++ = Xn1;
            *temp_pState++ = Xn2;
            *temp_pState++ = Yn1;
            *temp_pState++ = Yn2;

            /*  The first stage goes from the input buffer to the output buffer. */
            /*  Subsequent numStages  occur in-place in the output buffer */
            pIn = pDst;

            /* Reset the output pointer */
            pOut = pDst;

            /* decrement the loop counter */
            stage--;

        } while(stage > 0u);
    }
};






}


#endif 


