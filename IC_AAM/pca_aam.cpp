
/*
    Copyright (C) 2009-2010 Rohan Anil (rohan.anil@gmail.com) , Dr. Radhika Vathsan
    BITS Pilani Goa Campus
    http://code.google.com/p/aam-opencv/

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "stdafx.h"

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"
#include "pca_aam.h"
#include "aam.h"
PCA_AAM::PCA_AAM()
{

}


CvMat * PCA_AAM::returnEigenVals()	//	return eigenValMat;
{
    return eigenValMat;

}
void PCA_AAM::runPCA(	CvMat ** dataset, CvMat * meanXY_PreProcess, 
											int count, 
											int kEigens, 
											CvMat ** dataset_test, 
											CvMat * meanXY_TestImg_PreProcess,
											int count_test
											)	//	共 count 張 的 人臉訓練影像
{
    if (count>0)
        {}
    else
        return;
		
    int datasetSize = dataset[0]->width*dataset[0]->height;
	
    nEigens = (count > datasetSize) ? datasetSize : (count);


    CvMat* tmpEigenValues = cvCreateMat(1, nEigens, CV_64FC1);
    CvMat* tmpEigenVectors = cvCreateMat(nEigens, datasetSize, CV_64FC1);
    CvMat* MeanShape = cvCreateMat(1, datasetSize, CV_64FC1 );
    cvZero(MeanShape);

    CvMat * inputData=cvCreateMat(count, datasetSize, CV_64FC1);

    int i;
    for (i = 0; i < count; i ++)
    {
        for (int n=0;n<dataset[i]->height;n++)
        {

            for (int l=0;l<dataset[i]->width;l++)
            {


                CvScalar s;
                s=cvGet2D(dataset[i],n,l);
                cvSet2D(inputData,i,(n*dataset[i]->width) + l,s);
                CvScalar t = cvGet2D(MeanShape,0,(n*dataset[i]->width) + l);
                t.val[0]+=s.val[0];
                cvSet2D(MeanShape,0,(n*dataset[i]->width) + l,t);


            }
        }
    }
    for (int n=0;n<dataset[0]->height;n++)
    {

        for (int l=0;l<dataset[0]->width;l++)
        {

            CvScalar t = cvGet2D(MeanShape,0,(n*dataset[0]->width) + l);
            t.val[0]/=count;
            cvSet2D(MeanShape,0,(n*dataset[0]->width) + l,t);

        }
    }

    cvCalcPCA(inputData, MeanShape,tmpEigenValues, tmpEigenVectors, CV_PCA_DATA_AS_ROW);	//	建出 Space
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////// Using The Training Faces' Weights To Rescrount The Testing Faces /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (count_test > 0)
        {}
    else
        return;
		
    int datasetSize_test = dataset_test[0]->width * dataset_test[0]->height;


    CvMat* MeanShape_test = cvCreateMat(1, datasetSize_test, CV_64FC1 );
    cvSetZero(MeanShape_test);

    CvMat * inputData_test = cvCreateMat(count_test, datasetSize_test, CV_64FC1);

    for (int i=0; i<count_test; i++)
    {
        for (int n=0;n<dataset_test[i]->height;n++)
        {

            for (int l=0;l<dataset_test[i]->width;l++)
            {


                CvScalar s;
                s = cvGet2D(dataset_test[i], n, l);
                cvSet2D(inputData_test,i,(n*dataset_test[i]->width) + l, s);
                CvScalar t = cvGet2D(MeanShape_test, 0, (n*dataset_test[i]->width) + l);
                t.val[0] += s.val[0];
                cvSet2D(MeanShape_test, 0, (n*dataset_test[i]->width) + l, t);


            }
        }
    }
    for (int n = 0; n < dataset_test[0]->height; n ++)
    {

        for (int l = 0; l < dataset_test[0]->width; l ++)
        {

            CvScalar t = cvGet2D(MeanShape_test, 0, (n*dataset_test[0]->width) + l);
            t.val[0] /= count_test;
            cvSet2D(MeanShape_test, 0, (n*dataset_test[0]->width) + l, t);

        }
    }



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	CvMat* pWeight = cvCreateMat( count_test, kEigens, CV_32FC1 );
	//	count: 总的样本数, kEigens : PCA变换后的样本维数(即主成份的数目)
	
	cvProjectPCA( inputData_test, MeanShape_test, tmpEigenVectors, pWeight );	//	pResult : Project To Space

	//	重构,结果保存在pRecon中

	CvMat* pRecon = cvCreateMat( count_test, datasetSize_test, CV_32FC1 );		//	总的样本数, 每个样本的维数
	cvBackProjectPCA( pWeight, MeanShape_test, tmpEigenVectors, pRecon );		//	app:  pRecon( 105 * 10962 )

	//	Recover the inputData and then Save Image
	CvMat* reconMat = cvCreateMat( dataset_test[0]->height, dataset_test[0]->width, CV_32FC1 );
	CvMat* reconImg = cvCreateMat( 170, 120, CV_32FC3 );	//	cvCreateMat( count, dataset_test[0]->width, CV_32FC1 );

	char reconName[80];
	if(datasetSize < 1000)
	{
		/////////////////////////////////////Recover Shape Image////////////////////////////////////////////////////////
		for (int n = 0; n < pRecon->height; n ++)
		{
			CvScalar meanX_PreProcess, meanY_PreProcess;
			meanX_PreProcess = cvGet2D(meanXY_TestImg_PreProcess, n, 0);
			meanY_PreProcess = cvGet2D(meanXY_TestImg_PreProcess, n, 1);

			cvSetZero(reconImg);

			//printf("===========Face%d===========\n", n + 1);

			for (int l = 0, h = 0; l < pRecon->width; l += 2)
			{
				CvScalar s1, s2;
				
				s1 = cvGet2D(pRecon, n, l);
				//s1.val[0] = (s1.val[0] < 0) ? 0 : s1.val[0];

				s2 = cvGet2D(pRecon, n, l + 1);
				//s2.val[0] = (s2.val[0] < 0) ? 0 : s2.val[0];
				
				//abs();

				s1.val[0] += meanX_PreProcess.val[0];
				s2.val[0] += meanY_PreProcess.val[0];

				//printf("x = %lf, y = %lf\n", s1.val[0], s2.val[0]);

				cvCircle(reconImg, cvPoint( s1.val[0] , s2.val[0]  ), 1, CV_RGB(255, 255, 0), 2);
				
			}
			sprintf(reconName, "Test Face Shape Reconstruction/%d.jpg",  n+1);
			cvSaveImage(reconName, reconImg);
		}
		//printf("=============================\n\n");
		printf("\nFace Shape Reconstruction Completed ... \n");
	}
	else
	{
		/////////////////////////////////////Recover Appearance Image//////////////////////////////////////////////////
		for (int n = 0; n < pRecon->height; n ++)
		{
			for (int l = 0, h = 0; l < pRecon->width; l ++)
			{
				CvScalar s;
				s = cvGet2D(pRecon, n, l);
				if(  (l+1)  % dataset_test[0]->width)
					cvSet2D(reconMat, h ,  l - (h * dataset_test[0]->width), s);
				else{
					cvSet2D(reconMat, h,  l - (h * dataset_test[0]->width), s);
					h ++;
				}
			}
			sprintf(reconName, "Test Face Appearance Reconstruction/%d.jpg",  n+1);
			cvSaveImage(reconName, reconMat);
		}
		printf("\nFace Appearance Reconstruction Completed ... \n\n");
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    eigenValMat = cvCreateMat( 1, nEigens, CV_32FC1 );
    eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);
    for (i=0; i<nEigens; i++)
    {
        eigenVectArr[i] = cvCreateImage(cvSize(dataset[0]->width,dataset[0]->height), IPL_DEPTH_32F, 1);


        CvScalar s;
        s.val[0] = cvmGet(tmpEigenValues, 0, i);
        cvSet2D(eigenValMat,0,i,s);

    }

    averageInput = cvCreateImage(cvSize(dataset[0]->width,dataset[0]->height), IPL_DEPTH_32F, 1);
    
	for (i=0; i<nEigens; i++)
    {
        for (int n=0;n<dataset[i]->height;n++)
        {

            for (int l=0;l<dataset[i]->width;l++)
            {

                CvScalar s;

                s.val[0]= cvmGet(tmpEigenVectors,i,(n*dataset[i]->width) + l);
                cvSet2D(eigenVectArr[i],n,l,s);

            }
        }
    }
    for (int n=0;n<dataset[0]->height;n++)
    {
        for (int l=0;l<dataset[0]->width;l++)
        {

            CvScalar s;
            s= cvGet2D(MeanShape,0,(n*dataset[0]->width) + l);
            cvSet2D(averageInput,n,l,s);

        }
    }

    cvReleaseMat(&tmpEigenValues);
    cvReleaseMat(&tmpEigenVectors);
    cvReleaseMat(&inputData);
    cvReleaseMat(&MeanShape);
}
eigenVectors_AAM * PCA_AAM::returnEigens()
{
    eigenVectors_AAM * newEigen = (eigenVectors_AAM *) malloc(sizeof(eigenVectors_AAM));
    newEigen->count = nEigens;

    newEigen->eigens=eigenVectArr;
    return newEigen;

}
IplImage * PCA_AAM::returnAverage()	//	return averageInput;
{

    return averageInput;

}