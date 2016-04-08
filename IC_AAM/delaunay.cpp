
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

#include<stdio.h>
#include<math.h>

#include"aam.h"
#include"delaunay.h"

int delaunay::findInMeanShape(double x,double y)	//	檢查 特徵點 是否在 Edge 的 頭、尾 座標 上
{
    CvScalar x1,y1;
    x1=cvGet2D(meanShape,0,0);
    y1=cvGet2D(meanShape,0,1);

    double minx =fabs(x1.val[0]-x);
    double miny =fabs(y1.val[0]-y);
    double min = minx+miny;
    double indx=-1,indy=-1;
    double ind=-1;
    for (int i=0;i<meanShape->height;i++)
    {
        x1=cvGet2D(meanShape,i,0);
        y1=cvGet2D(meanShape,i,1);
        x1.val[0]+=tx;
        y1.val[0]+=ty;
        if (minx>fabs(x1.val[0]-x))
        {
            minx=fabs(x1.val[0]-x);
            indx=i;
        }

        if (miny>fabs(y1.val[0]-y))
        {
            indy=i;

            miny=fabs(y1.val[0]-y);
        }
        if ((fabs(y1.val[0]-y)+ fabs(x1.val[0]-x))<=min)
        {
            min= fabs(y1.val[0]-y)+ fabs(x1.val[0]-x);
            ind=i;
        }
        if (x1.val[0]==x && y1.val[0]==y)
        {
            return i;
        }
        else
        {
        }
    }
    if (min<5E-1F)
        return ind;
    return -1;
}

void delaunay::draw_subdiv_facet( IplImage* img, CvSubdiv2DEdge edge, int k )
{
    CvSubdiv2DEdge t = edge;
    int i, count = 0;
    CvPoint* buf = 0;
    int arr[3];	//	記錄鄰近的3個特徵點，可圍成一個三角形
    int g=0;

    do
    {
        CvSubdiv2DPoint* a = cvSubdiv2DEdgeDst(t);	//	edge 的 尾座標
        CvSubdiv2DPoint* b = cvSubdiv2DEdgeOrg(t);	//	edge 的 頭座標
        if (findInMeanShape(a->pt.x,a->pt.y)>=0)	//	true : 則 特徵點 在 Edge_t 的 尾座標 上
        {

            if (findInMeanShape(b->pt.x,b->pt.y)>=0)	//	true : 則 特徵點 在 Edge_t 的 頭座標 上
            {
                arr[g]= findInMeanShape(a->pt.x,a->pt.y);	//	當 Edge_t 的 頭、尾座標上 皆已有特徵點，則將 Edge_t 的 尾座標 記錄下來
                g++;
                count++;
            }
        }
        t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
    }
    while (t != edge );

    for (i=0;i<3;i++)
    {
        for (int l=0;l<3-i-1;l++)
        {
            int m;

            if (arr[l]>arr[l+1])	//	按 編號 排序 3個 特徵點
            {
                m=arr[l];
                arr[l]=arr[l+1];
                arr[l+1]=m;
            }


        }
    }
    buf = (CvPoint*)malloc( count * sizeof(buf[0]));
    t = edge;	//	存取先前的 Edge
    if (count!=3)	//		Triangle not yet !
        return;

    for ( i = 0; i < count; i++ )
    {
        CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );
        if ( !pt ) break;
        buf[i] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));	//	取出 Edge 的 頭 座標
        t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
    }
    if ( i == count )	//	填滿三角形區域
    {
        colorTriangleCode[k][0]=arr[0];
        colorTriangleCode[k][1]=arr[1];
        colorTriangleCode[k][2]=arr[2];
        cvFillConvexPoly( img,buf, count,CV_RGB(k,k,k), 0, 0 );	//	cvFillConvexPoly(IplImage資料結構,CvPoint陣列,多邊型頂點數,CvScalar顏色,線條類型,比例縮放數據)
    }
    free( buf );
}

CvSubdiv2D* delaunay::init_delaunay( CvMemStorage* storage,
                                     CvRect rect )
{
    CvSubdiv2D* subdiv;

    subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
                               sizeof(CvSubdiv2DPoint),
                               sizeof(CvQuadEdge2D),
                               storage );
    cvInitSubdivDelaunay2D( subdiv, rect );

    return subdiv;
}

delaunay::delaunay(CvMat* shape)
{

    CvScalar minx,miny,maxx,maxy;
    minx=cvGet2D(shape,0,0);
    miny=cvGet2D(shape,0,1);
    maxx.val[0]=minx.val[0];
    maxy.val[0]=miny.val[0];
    meanShape = cvCreateMat(shape->height,2, CV_64FC1 );
    storage = cvCreateMemStorage(0);



    for (int i = 0; i <  shape->height; i++ )
    {


        CvScalar x,y;
        x=cvGet2D(shape,i,0);

        if (x.val[0]<minx.val[0])
            minx.val[0]=x.val[0];

        if (x.val[0]>maxx.val[0])
            maxx.val[0]=x.val[0];

        y=cvGet2D(shape,i,1);

        if (y.val[0]<miny.val[0])
            miny.val[0]=y.val[0];

        if (y.val[0]>maxy.val[0])
            maxy.val[0]=y.val[0];

        cvSet2D(meanShape,i,0,x);
        cvSet2D(meanShape,i,1,y);
    }

    tx=-1*minx.val[0];
    ty=-1*miny.val[0];
    rect.x=0;
    rect.y=0;

    rect.width=(maxx.val[0]-minx.val[0] +1);
    rect.height=(maxy.val[0]-miny.val[0] +1);

    width=rect.width;
    height=rect.height;	//	將 texture(appearance) 的外框，計算出來。 寬: width ; 高: height
    subdiv = init_delaunay( storage, rect );	//	初始化subdiv

    rasterImage=   cvCreateImage( cvSize( rect.width, rect.height),IPL_DEPTH_32F, 3);
    rasterImageColor=   cvCreateImage( cvSize( rect.width, rect.height),IPL_DEPTH_8U, 3);

    cvZero(rasterImage);
    cvZero(rasterImageColor);

    for (int i = 0; i <  shape->height; i++ )
    {
        CvScalar x,y;
        x=cvGet2D(shape,i,0);
        y=cvGet2D(shape,i,1);

        double x1 = (tx+ x.val[0]);
        double y1 = (ty+ y.val[0]);

        CvPoint2D32f fp = cvPoint2D32f(x1,y1);

        cvSubdivDelaunay2DInsert(subdiv,fp);	//	在指定區域(rect)中插入標記出所有平均特徵點，劃分出所有 ( Subdivsion Triangle )子區塊
    }

    CvSeqReader  reader;
    int i, total = subdiv->edges->total;	//	total of Traingle edges
    int elem_size = subdiv->edges->elem_size;
    cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );	//	存取 Edges 的 First Address 於 reader

    colorTriangleCode = (int **)malloc(total*4*sizeof(int*));
    for ( i = 0; i < 4*total; i++ )
    {
        colorTriangleCode[i]= (int *)malloc(3*sizeof(int));
    }

    int k=0;
    for ( i = 0; i < total; i++ )	//	Travel every Edge of Traingles
    {
        CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

        if ( CV_IS_SET_ELEM( edge ))	//	替 所有 ( Subdivsion Triangle )子區塊 各別 填滿 不同的 顏色
        {
            CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge;
            k++;	//	count Traingle edges == total
            draw_subdiv_facet( rasterImage, cvSubdiv2DRotateEdge( e, 0 ),k);	//	cvSubdiv2DRotateEdge : Return Next Edge. k = 1, 2, 3, 4, ..... 
            k++;																		//	0 the input edge ( e on the picture below if e is the input edge)
            draw_subdiv_facet( rasterImage, cvSubdiv2DRotateEdge( e, 2 ),k);	//	cvSubdiv2DRotateEdge : Return Next Edge. 
        }																				//	2 the reversed edge (reversed e (in green))
        CV_NEXT_SEQ_ELEM( elem_size, reader );	//	往後挪取 Pointer_Edges 指針 於 下一個Address 存於 reader
    }

    int * arrayT=(int *)malloc(k*sizeof(int));

    int count=1;


    for (i=0;i<k;i++)	// Array for Recording the Traingles Number
        arrayT[i]=0;
    int pixelCount=0;
    for (i=0;i<rasterImage->height;i++)
    {

        for (int j=0;j<rasterImage->width;j++)
        {
            CvScalar s;
            s=cvGet2D(rasterImage,i,j);
            if (s.val[0]!=0)	//	True : Color point
            {
                pixelCount++;
                if (arrayT[int(s.val[0])-1]==0)	// True : Different Color , s.val[0] = 1, 2, 3, 4, ..... = count
                {
                    arrayT[int(s.val[0])-1]=count;	// count = 1, 2, 3, 4, ..... = arrayT[int(s.val[0])-1]	替 所有 Triangles 編號
                    s.val[0]=count;
                    count++;			//	著上顏色種數(Triangle總數) = count - 1
                }
                else	//	Same Color
                {
                    s.val[0]=arrayT[int(s.val[0])-1];	//	maintain the same color
                }

            }

            s.val[1]=0;	//	RGB = (count,0,0)
            s.val[2]=0;

			//	製作 rasterImageColor
            if (s.val[0])	//	True : Color point
            {
                CvScalar s3;	//	 cvScalar( (b), (g), (r), 0 )
                s3.val[0]=s.val[0];
                s3.val[1] =1.5*s.val[0];

                s3.val[0] =255 -10*s.val[0];	//	"s3.val[0]" should be modified to "s3.val[2]"   ?

                cvSet2D(rasterImageColor,i,j,s3);
            }

            cvSet2D(rasterImage,i,j,s);
        }
    }

    totalNumberOfPixels=pixelCount;	//	Total of Color Points (Pixels)  即 Triangle 個數
    pixelCode = (double **)malloc((totalNumberOfPixels)*sizeof(double*));
    for (int m=0;m<pixelCount;m++)
        pixelCode[m] = (double *)malloc((3)*sizeof(double));

    int in=0;

    for (i=0;i<rasterImage->height;i++)
    {

        for (int j=0;j<rasterImage->width;j++)
        {

            CvScalar s;
            s=cvGet2D(rasterImage,i,j);
            if (s.val[0]!=0)	//	記錄 Color point & It's coodernate
            {
                pixelCode[in][0]=s.val[0];
                pixelCode[in][1]=i;
                pixelCode[in][2]=j;

                in++;

            }
        }

    }



    colorTriangleCodeFixed = (int **)malloc((count)*sizeof(int*));	//	count = 著上顏色種數 + 1

    for (i=0;i<k;i++)
    {
        if (arrayT[i]>0)
        {
            colorTriangleCodeFixed[arrayT[i]]= colorTriangleCode[i+1];


        }
    }





    ithNodeCode = (barycentrics*)malloc(sizeof(barycentrics)* (shape->height));

    barycentric temp[20];


    int ind=0;

    for ( int w = 0; w < shape->height; w++ )
    {
        ind =0;
        for (int j=0;j<3;j++)
        {
            for ( i = 1; i < count; i++ )	//	count = 著上顏色種數(Triangle總數) + 1
            {
                int m =  colorTriangleCodeFixed[i][j];
                int kk,jj;
                if (m==w)
                {
                    CvScalar x = cvGet2D(shape,w,0);
                    CvScalar y = cvGet2D(shape,w,1);

                    if (j==0)
                    {
                        kk=1;
                        jj=2;

                    }
                    else if (j==1)
                    {
                        kk=0;
                        jj=2;
                    }
                    else if (j==2)
                    {
                        kk=0;
                        jj=1;
                    }
                    int kthnode =  colorTriangleCodeFixed[i][kk];
                    int jthnode =  colorTriangleCodeFixed[i][jj];
                    CvScalar xk = cvGet2D(shape,kthnode,0);
                    CvScalar yk = cvGet2D(shape,kthnode,1);
                    CvScalar xj = cvGet2D(shape,jthnode,0);
                    CvScalar yj = cvGet2D(shape,jthnode,1);
                    temp[ind].diff_xj_xi= xj.val[0] - x.val[0];
                    temp[ind].diff_yj_yi= yj.val[0] - y.val[0];
                    temp[ind].diff_xk_xi= xk.val[0] - x.val[0];
                    temp[ind].diff_yk_yi= yk.val[0] - y.val[0];
                    temp[ind].xi=x.val[0];
                    temp[ind].yi=y.val[0];
                    temp[ind].colorcode=i;
                    ind++;
                }
            }
        }
        ithNodeCode[w].triangles =  (barycentric*)malloc(sizeof(barycentric)* (ind));
        for (int b =0 ;b<ind ;b++)
        {
            ithNodeCode[w].triangles[b].diff_xj_xi =     temp[b].diff_xj_xi;
            ithNodeCode[w].triangles[b].diff_yj_yi =     temp[b].diff_yj_yi;
            ithNodeCode[w].triangles[b].diff_xk_xi =     temp[b].diff_xk_xi;
            ithNodeCode[w].triangles[b].diff_yk_yi =     temp[b].diff_yk_yi;
            ithNodeCode[w].triangles[b].xi         =     temp[b].xi;
            ithNodeCode[w].triangles[b].yi         =     temp[b].yi;
            ithNodeCode[w].triangles[b].colorcode  =     temp[b].colorcode;
            ithNodeCode[w].triangles[b].affine[0]  =     1;
            ithNodeCode[w].triangles[b].affine[1]  =     0;
            ithNodeCode[w].triangles[b].affine[2]  =     0;
            ithNodeCode[w].triangles[b].affine[3]  =     0;
            ithNodeCode[w].triangles[b].affine[4]  =     1;
            ithNodeCode[w].triangles[b].affine[5]  =     0;


            ithNodeCode[w].count                    =     ind;
            ithNodeCode[w].nodenumber                     =     w;


        }
    }
    totalNumberofTriangles=count-1;
    affineInverse=(CvMat**)cvAlloc(sizeof(CvMat*) *totalNumberofTriangles);
    for (int i = 1; i <= totalNumberofTriangles; i++ )
    {

        int node1=colorTriangleCodeFixed[i][0];
        int node2=colorTriangleCodeFixed[i][1];
        int node3=colorTriangleCodeFixed[i][2];
        CvScalar mx1,my1;
        CvScalar mx2,my2;
        CvScalar mx3,my3;
        mx1=cvGet2D(meanShape,node1,0);
        my1=cvGet2D(meanShape,node1,1);
        mx2=cvGet2D(meanShape,node2,0);
        my2=cvGet2D(meanShape,node2,1);
        mx3=cvGet2D(meanShape,node3,0);
        my3=cvGet2D(meanShape,node3,1);
        CvMat * A = cvCreateMat(3,3,CV_64FC1 );
        affineInverse[i-1]=cvCreateMat(3,3,CV_64FC1 );
        CvScalar one;
        one.val[0]=1;
        cvSet2D(A,0,0,mx1);
        cvSet2D(A,1,0,mx2);
        cvSet2D(A,2,0,mx3);

        cvSet2D(A,0,1,my1);
        cvSet2D(A,1,1,my2);
        cvSet2D(A,2,1,my3);

        cvSet2D(A,0,2,one);
        cvSet2D(A,1,2,one);
        cvSet2D(A,2,2,one);

        cvInvert(A,affineInverse[i-1],CV_LU);
        cvReleaseMat(&A);
    }

    warpTriangleCode=(double **)malloc((totalNumberofTriangles+1)*sizeof(double*));
    for (int m=0;m<totalNumberofTriangles+1;m++)
        warpTriangleCode[m]=(double *)malloc((6)*sizeof(double));

    //showRasterImage();



}
void delaunay:: showRasterImage()
{
    cvNamedWindow("test1",1);

    cvNamedWindow("test",1);
    cvShowImage("test",rasterImage);
    cvShowImage("test1",rasterImageColor);

    //cvWaitKey(-1);

}
/*aamImage*
delaunay:: warpImageToMeanShape(aamImage*input)
{

}
*/
void
delaunay:: calculateAffineWarpParameters(CvMat *targetVectors)
{

    static  CvMat * B1 = cvCreateMat(3,1,CV_64FC1 );
    static CvMat * B2 = cvCreateMat(3,1,CV_64FC1 );

    static CvMat * affine1 = cvCreateMat(3,1,CV_64FC1 );
    static CvMat * affine2 = cvCreateMat(3,1,CV_64FC1 );


    for (int i = 1; i <= totalNumberofTriangles; i++ )
    {
        int node1=colorTriangleCodeFixed[i][0];
        int node2=colorTriangleCodeFixed[i][1];
        int node3=colorTriangleCodeFixed[i][2];
        B1->data.db[ (0) *B1->cols + 0 ] =     targetVectors->data.db[ (node1) *targetVectors->cols + 0 ];
        B1->data.db[ (1) *B1->cols + 0 ] =     targetVectors->data.db[ (node2) *targetVectors->cols + 0 ];
        B1->data.db[ (2) *B1->cols + 0 ] =     targetVectors->data.db[ (node3) *targetVectors->cols + 0 ];


        B2->data.db[ (0) *B2->cols + 0 ] =     targetVectors->data.db[ (node1) *targetVectors->cols + 1 ];
        B2->data.db[ (1) *B2->cols + 0 ] =     targetVectors->data.db[ (node2) *targetVectors->cols + 1 ];
        B2->data.db[ (2) *B2->cols + 0 ] =     targetVectors->data.db[ (node3) *targetVectors->cols + 1 ];


        cvMatMul(affineInverse[i-1],B1,affine1);
        cvMatMul(affineInverse[i-1],B2,affine2);

        warpTriangleCode[i][0] = affine1->data.db[ (0) *affine1->cols + 0 ];
        warpTriangleCode[i][1] = affine1->data.db[ (1) *affine1->cols + 0 ];
        warpTriangleCode[i][2] = affine1->data.db[ (2) *affine1->cols + 0 ];
        warpTriangleCode[i][3] = affine2->data.db[ (0) *affine2->cols + 0 ];
        warpTriangleCode[i][4] = affine2->data.db[ (1) *affine2->cols + 0 ];
        warpTriangleCode[i][5] = affine2->data.db[ (2) *affine2->cols + 0 ];
        int w,ind;

        w=node1;	// 於 meanShape 的該點(編號[w]) 
        ind = ithNodeCode[w].count;	// 於 meanShape 的該點(編號[w]) 圍成 每一個 Triangle 的 次數
        for (int b =0 ;b<ind ;b++)
        {
            if (ithNodeCode[w].triangles[b].colorcode==i)
            {
                //  printf("%d true \n");
                ithNodeCode[w].triangles[b].affine[0]  =     warpTriangleCode[i][0];
                ithNodeCode[w].triangles[b].affine[1]  =      warpTriangleCode[i][1];
                ithNodeCode[w].triangles[b].affine[2]  =     warpTriangleCode[i][2];
                ithNodeCode[w].triangles[b].affine[3]  =      warpTriangleCode[i][3];
                ithNodeCode[w].triangles[b].affine[4]  =      warpTriangleCode[i][4];
                ithNodeCode[w].triangles[b].affine[5]  =       warpTriangleCode[i][5];
            }


        }

        w=node2;
        ind = ithNodeCode[w].count;
        for (int b =0 ;b<ind ;b++)
        {
            if (ithNodeCode[w].triangles[b].colorcode==i)
            {
                ithNodeCode[w].triangles[b].affine[0]  =     warpTriangleCode[i][0];
                ithNodeCode[w].triangles[b].affine[1]  =      warpTriangleCode[i][1];
                ithNodeCode[w].triangles[b].affine[2]  =     warpTriangleCode[i][2];
                ithNodeCode[w].triangles[b].affine[3]  =      warpTriangleCode[i][3];
                ithNodeCode[w].triangles[b].affine[4]  =      warpTriangleCode[i][4];
                ithNodeCode[w].triangles[b].affine[5]  =       warpTriangleCode[i][5];
            }


        }

        w=node3;
        ind = ithNodeCode[w].count;
        for (int b =0 ;b<ind ;b++)
        {
            if (ithNodeCode[w].triangles[b].colorcode==i)
            {
                ithNodeCode[w].triangles[b].affine[0]  =     warpTriangleCode[i][0];
                ithNodeCode[w].triangles[b].affine[1]  =      warpTriangleCode[i][1];
                ithNodeCode[w].triangles[b].affine[2]  =     warpTriangleCode[i][2];
                ithNodeCode[w].triangles[b].affine[3]  =      warpTriangleCode[i][3];
                ithNodeCode[w].triangles[b].affine[4]  =      warpTriangleCode[i][4];
                ithNodeCode[w].triangles[b].affine[5]  =       warpTriangleCode[i][5];
            }


        }

    }

}

void
delaunay:: calculateAffineWarpParametersComposeWithCurrent(CvMat *targetVectors)
{

    static  CvMat * B1 = cvCreateMat(3,1,CV_64FC1 );
    static CvMat * B2 = cvCreateMat(3,1,CV_64FC1 );

    static CvMat * affine1 = cvCreateMat(3,1,CV_64FC1 );
    static CvMat * affine2 = cvCreateMat(3,1,CV_64FC1 );
    static CvMat * AFF1 = cvCreateMat(3,3,CV_64FC1 );
    static  CvMat * AFF2 = cvCreateMat(3,3,CV_64FC1 );
    for (int i = 1; i <= totalNumberofTriangles; i++ )
    {


        int node1=colorTriangleCodeFixed[i][0];
        int node2=colorTriangleCodeFixed[i][1];
        int node3=colorTriangleCodeFixed[i][2];





        B1->data.db[ (0) *B1->cols + 0 ]=      targetVectors->data.db[ (node1) *targetVectors->cols + 0 ];
        B1->data.db[ (1) *B1->cols + 0 ]=       targetVectors->data.db[ (node2) *targetVectors->cols + 0 ];
        B1->data.db[ (2) *B1->cols + 0 ]=       targetVectors->data.db[ (node3) *targetVectors->cols + 0 ];


        B2->data.db[ (0) *B2->cols + 0 ]=       targetVectors->data.db[ (node1) *targetVectors->cols + 1 ];
        B2->data.db[ (1) *B2->cols + 0 ]=       targetVectors->data.db[ (node2) *targetVectors->cols + 1 ];
        B2->data.db[ (2) *B2->cols + 0 ]=       targetVectors->data.db[ (node3) *targetVectors->cols + 1 ];



        cvMatMul(affineInverse[i-1],B1,affine1);
        cvMatMul(affineInverse[i-1],B2,affine2);




        AFF2->data.db[ (0) *AFF2->cols + 0 ]=      affine1->data.db[ (0) *affine1->cols + 0 ];
        AFF2->data.db[ (0) *AFF2->cols + 1 ]=       affine1->data.db[ (1) *affine1->cols + 0 ];
        AFF2->data.db[ (0) *AFF2->cols + 2 ]=       affine1->data.db[ (2) *affine1->cols + 0 ];
        AFF2->data.db[ (1) *AFF2->cols + 0 ]=       affine2->data.db[ (0) *affine2->cols + 0 ];
        AFF2->data.db[ (1) *AFF2->cols + 1 ]=       affine2->data.db[ (1) *affine2->cols + 0 ];
        AFF2->data.db[ (1) *AFF2->cols + 2 ]=       affine2->data.db[ (2) *affine2->cols + 0 ];

        AFF2->data.db[ (2) *AFF2->cols + 0 ]= 0;
        AFF2->data.db[ (2) *AFF2->cols + 1 ]= 0;
        AFF2->data.db[ (2) *AFF2->cols + 2 ]= 1;





        AFF1->data.db[ (0) *AFF1->cols + 0 ]=  warpTriangleCode[i][0];
        AFF1->data.db[ (0) *AFF1->cols + 1 ]=  warpTriangleCode[i][1];
        AFF1->data.db[ (0) *AFF1->cols + 2 ]=  warpTriangleCode[i][2];
        AFF1->data.db[ (1) *AFF1->cols + 0 ]=  warpTriangleCode[i][3];
        AFF1->data.db[ (1) *AFF1->cols + 1 ]=  warpTriangleCode[i][4];
        AFF1->data.db[ (1) *AFF1->cols + 2 ]=  warpTriangleCode[i][5];

        AFF1->data.db[ (2) *AFF1->cols + 0 ]= 0;
        AFF1->data.db[ (2) *AFF1->cols + 1 ]= 0;
        AFF1->data.db[ (2) *AFF1->cols + 2 ]= 1;

        cvMatMul(AFF2,AFF1,AFF1);

        warpTriangleCode[i][0]= AFF1->data.db[ (0) *AFF1->cols + 0 ];
        warpTriangleCode[i][1]= AFF1->data.db[ (0) *AFF1->cols + 1 ];
        warpTriangleCode[i][2]=AFF1->data.db[ (0) *AFF1->cols + 2];
        warpTriangleCode[i][3]=AFF1->data.db[ (1) *AFF1->cols + 0 ];
        warpTriangleCode[i][4]=AFF1->data.db[ (1) *AFF1->cols + 1 ];
        warpTriangleCode[i][5]=AFF1->data.db[ (1) *AFF1->cols + 2 ];



        int w,ind;


        w=node1;
        ind = ithNodeCode[w].count;
        for (int b =0 ;b<ind ;b++)
        {
            if (ithNodeCode[w].triangles[b].colorcode==i)
            {
                //  printf("%d true \n");
                ithNodeCode[w].triangles[b].affine[0]  =     warpTriangleCode[i][0];
                ithNodeCode[w].triangles[b].affine[1]  =      warpTriangleCode[i][1];
                ithNodeCode[w].triangles[b].affine[2]  =     warpTriangleCode[i][2];
                ithNodeCode[w].triangles[b].affine[3]  =      warpTriangleCode[i][3];
                ithNodeCode[w].triangles[b].affine[4]  =      warpTriangleCode[i][4];
                ithNodeCode[w].triangles[b].affine[5]  =       warpTriangleCode[i][5];
            }


        }

        w=node2;
        ind = ithNodeCode[w].count;
        for (int b =0 ;b<ind ;b++)
        {
            if (ithNodeCode[w].triangles[b].colorcode==i)
            {
                ithNodeCode[w].triangles[b].affine[0]  =     warpTriangleCode[i][0];
                ithNodeCode[w].triangles[b].affine[1]  =      warpTriangleCode[i][1];
                ithNodeCode[w].triangles[b].affine[2]  =     warpTriangleCode[i][2];
                ithNodeCode[w].triangles[b].affine[3]  =      warpTriangleCode[i][3];
                ithNodeCode[w].triangles[b].affine[4]  =      warpTriangleCode[i][4];
                ithNodeCode[w].triangles[b].affine[5]  =       warpTriangleCode[i][5];
            }


        }

        w=node3;
        ind = ithNodeCode[w].count;
        for (int b =0 ;b<ind ;b++)
        {
            if (ithNodeCode[w].triangles[b].colorcode==i)
            {
                ithNodeCode[w].triangles[b].affine[0]  =     warpTriangleCode[i][0];
                ithNodeCode[w].triangles[b].affine[1]  =      warpTriangleCode[i][1];
                ithNodeCode[w].triangles[b].affine[2]  =     warpTriangleCode[i][2];
                ithNodeCode[w].triangles[b].affine[3]  =      warpTriangleCode[i][3];
                ithNodeCode[w].triangles[b].affine[4]  =      warpTriangleCode[i][4];
                ithNodeCode[w].triangles[b].affine[5]  =       warpTriangleCode[i][5];
            }


        }


    }

}


pixel*
delaunay:: getpixel(int num)	//	取得該點 warp 以後，所對應的數值
{
    if (num>=totalNumberOfPixels)
        return NULL;

    static pixel* newPixel=(pixel*)malloc(sizeof(pixel));
    newPixel->x=pixelCode[num][2]-tx;	//	[1] ?   tx = (-1) * minx
    newPixel->y=pixelCode[num][1]-ty;	//	[2] ?   ty = (-1) * miny
    newPixel->tx=tx;
    newPixel->ty=ty;
    newPixel->a1=warpTriangleCode[(int)pixelCode[num][0]][0];
    newPixel->a2=warpTriangleCode[(int)pixelCode[num][0]][1];
    newPixel->a3=warpTriangleCode[(int)pixelCode[num][0]][2];
    newPixel->a4=warpTriangleCode[(int)pixelCode[num][0]][3];
    newPixel->a5=warpTriangleCode[(int)pixelCode[num][0]][4];
    newPixel->a6=warpTriangleCode[(int)pixelCode[num][0]][5];
    newPixel->type=pixelCode[num][0];
    newPixel->width=width;
    newPixel->height=height;
    return newPixel;

}


int
delaunay::numberOfPixels()
{
    return totalNumberOfPixels;

}


pixel*
delaunay::findCorrespondingPixelInImage(pixel* pix)	//	找出 對應點 的 X 座標 和 Y 座標
{

    double dx = pix->a1 * pix->x  + pix->a2 * pix->y + pix->a3;
    double dy = pix->a4 * pix->x  + pix->a5 * pix->y + pix->a6;


    static pixel* newPixel = ( pixel* )malloc(sizeof(pixel));
    newPixel->x=dx;	//	***
    newPixel->y=dy;	//	***
    newPixel->tx=0;
    newPixel->ty=0;

    newPixel->a1=0;
    newPixel->a2=0;
    newPixel->a3=0;
    newPixel->a4=0;
    newPixel->a5=0;
    newPixel->a6=0;
    newPixel->width=0;
    newPixel->height=0;
    return newPixel;

}


