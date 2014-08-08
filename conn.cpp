//______________________________________________________________________________________
// Program : OpenCV connected component analysis
// Author  : Bharath Prabhuswamy
//______________________________________________________________________________________
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B); 	// Routine to update Bounding Box corners

// Start of Main Loop
//------------------------------------------------------------------------------------------------------------------------
int main ( int argc, char **argv )
{
	CvCapture* capture = 0;
	IplImage* img = 0;

	capture = cvCaptureFromCAM( 0 );
		if ( !capture )           					// Check for Camera capture
		return -1;

	cvNamedWindow("Camera",CV_WINDOW_AUTOSIZE);

	//cvNamedWindow("Threshold",CV_WINDOW_AUTOSIZE);

	// cvNamedWindow("Test",CV_WINDOW_AUTOSIZE);	// Test window to push any visuals during debugging

	IplImage* gray = 0;
	IplImage* thres = 0;
	IplImage* prcs_flg = 0;					// Process flag to flag whether the current pixel is already processed as part blob detection


	int q,i;								// Intermidiate variables
	int h,w;								// Variables to store Image Height and Width

	int ihist[256];     	                // Array to store Histogram values
	float hist_val[256];					// Array to store Normalised Histogram values

	int blob_count;
	int n;                                	// Number of pixels in a blob
	int pos ;								// Position or pixel value of the image

	int rectw,recth;                    	// Width and Height of the Bounding Box
	double aspect_ratio; 					// Aspect Ratio of the Bounding Box

	int min_blob_sze = 400;              	// Minimum Blob size limit 
	int max_blob_sze = 150000;           	// Maximum Blob size limit


	bool init = false;						// Flag to identify initialization of Image objects


	//Step	: Capture a frame from Camera for creating and initializing manipulation variables
	//Info	: Inbuit functions from OpenCV
	//Note	: 

    	if(init == false)
	{
        	img = cvQueryFrame( capture );	// Query for the frame
        		if( !img )		// Exit if camera frame is not obtained
			return -1;

		// Creation of Intermediate 'Image' Objects required later
		gray = cvCreateImage( cvGetSize(img), 8, 1 );		// To hold Grayscale Image
		thres = cvCreateImage( cvGetSize(img), 8, 1 );		// To hold OTSU thresholded Image
		prcs_flg = cvCreateImage( cvGetSize(img), 8, 1 );	// To hold Map of 'per Pixel' Flag to keep track while identifing Blobs
		
		init = true;
	}

	int clr_flg[img->width];		// Array representing elements of entire current row to assign Blob number
	int clrprev_flg[img->width];	// Array representing elements of entire previous row to assign Blob number

	h = img->height;		// Height and width of the Image
	w = img->width;

	int key = 0;
	while(key != 'q')		// While loop to query for Camera frame
	{
	   
		//Step	: Capture Image from Camera
		//Info	: Inbuit function from OpenCV
		//Note	: 

		img = cvQueryFrame( capture );		// Query for the frame

		//Step	: Convert Image captured from Camera to GrayScale
		//Info	: Inbuit function from OpenCV
		//Note	: Image from Camera and Grayscale are held using seperate "IplImage" objects

		cvCvtColor(img,gray,CV_RGB2GRAY);	// Convert RGB image to Gray


		//Step	: Threshold the image using optimum Threshold value obtained from OTSU method
		//Info	: 
		//Note	: 

		memset(ihist, 0, 256);

		for(int j = 0; j < gray->height; ++j)	// Use Histogram values from Gray image
		{
			uchar* hist = (uchar*) (gray->imageData + j * gray->widthStep);
			for(int i = 0; i < gray->width; i++ )
			{
				pos = hist[i];		// Check the pixel value
				ihist[pos] += 1;	// Use the pixel value as the position/"Weight"
			}
		}

		//Parameters required to calculate threshold using OTSU Method
		float prbn = 0.0;                   // First order cumulative
		float meanitr = 0.0;                // Second order cumulative
		float meanglb = 0.0;                // Global mean level
		int OPT_THRESH_VAL = 0;             // Optimum threshold value
		float param1,param2;                // Parameters required to work out OTSU threshold algorithm
		double param3 = 0.0;

		//Normalise histogram values and calculate global mean level
		for(int i = 0; i < 256; ++i)
		{
			hist_val[i] = ihist[i] / (float)(w * h);
			meanglb += ((float)i * hist_val[i]);
		}

	    	// Implementation of OTSU algorithm
		for (int i = 0; i < 255; i++)
		{
			prbn += (float)hist_val[i];
			meanitr += ((float)i * hist_val[i]);

			param1 = (float)((meanglb * prbn) - meanitr);
			param2 = (float)(param1 * param1) /(float) ( prbn * (1.0f - prbn) );

			if (param2 > param3)
			{
			    param3 = param2;
			    OPT_THRESH_VAL = i; 				// Update the "Weight/Value" as Optimum Threshold value
			}
		}

		cvThreshold(gray,thres,OPT_THRESH_VAL,255,CV_THRESH_BINARY);	//Threshold the Image using the value obtained from OTSU method


		//Step	: Identify Blobs in the OTSU Thresholded Image
		//Info	: Custom Algorithm to Identify blobs
		//Note	: This is a complicated method. Better refer the presentation, documentation or the Demo

		blob_count = 0;				// Current Blob number used to represent the Blob
		CvPoint cornerA,cornerB; 	// Two Corners to represent Bounding Box

		memset(clr_flg, 0, w);		// Reset all the array elements ; Flag for tracking progress
		memset(clrprev_flg, 0, w);

		cvZero(prcs_flg);			// Reset all Process flags


        for( int y = 0; y < thres->height; ++y)	//Start full scan of the image by incrementing y
        {
            uchar* prsnt = (uchar*) (thres->imageData + y * thres->widthStep);
            uchar* pntr_flg = (uchar*) (prcs_flg->imageData + y * prcs_flg->widthStep);  // pointer to access the present value of pixel in Process flag
			uchar* scn_prsnt;      // pointer to access the present value of pixel related to a particular blob
			uchar* scn_next;       // pointer to access the next value of pixel related to a particular blob

            for(int x = 0; x < thres->width; ++x )	//Start full scan of the image by incrementing x
            {
                int c = 0;					// Number of edgels in a particular blob
               
                if((prsnt[x] == 0) && (pntr_flg [x] == 0))	// If current pixel is black and has not been scanned before - continue
                {
			blob_count +=1;                          // Increment at the start of processing new blob
			clr_flg [x] = blob_count;                // Update blob number
			pntr_flg [x] = 255;                      // Mark the process flag

			n = 1;                                   // Update pixel count of this particular blob / this iteration

			cornerA.x = x;                           // Update Bounding Box Location for this particular blob / this iteration
			cornerA.y = y;
			cornerB.x = x;
			cornerB.y = y;

			int lx,ly;				// Temp location to store the initial position of the blob
			int belowx = 0;

			bool checkbelow = true;			// Scan the below row to check the continuity of the blob

                    ly=y;

                    bool below_init = 1;					// Flags to facilitate the scanning of the entire blob once
                    bool start = 1;

                        while(ly < h)						// Start the scanning of the blob
                        {
                            if(checkbelow == true)			// If there is continuity of the blob in the next row & checkbelow is set; continue to scan next row
                            {
                                if(below_init == 1) 		// Make a copy of Scanner pixel position once / initially
                                {
                                    belowx=x;
                                    below_init = 0;
                                }

                                checkbelow = false;		// Clear flag before next flag

                                scn_prsnt = (uchar*) (thres->imageData + ly * thres->widthStep);
                                scn_next = (uchar*) (thres->imageData + (ly+1) * thres->widthStep);

                                pntr_flg = (uchar*) (prcs_flg->imageData + ly * prcs_flg->widthStep);

                                bool onceb = 1;			// Flag to set and check blbo continuity for next row

                                //Loop to move Scanner pixel to the extreme left pixel of the blob
                                while((scn_prsnt[belowx-1] == 0) && ((belowx-1) > 0) && (pntr_flg[belowx-1]== 0))
                                {
                                    cv_adjustBox(belowx,ly,cornerA,cornerB);    // Update Bounding Box corners
                                    pntr_flg [belowx] = 255;

                                    clr_flg [belowx] = blob_count;

                                    n = n+1;
                                    belowx--;
                                }
                                //Scanning of a particular row of the blob
                                for(lx = belowx; lx < thres->width; ++lx )
                                {
                                    if(start == 1)                 	// Initial/first row scan
                                    {
                                        cv_adjustBox(lx,ly,cornerA,cornerB);
                                        pntr_flg [lx] = 255;

                                        clr_flg [lx] = blob_count;


                                        start = 0;
                                        if((onceb == 1) && (scn_next[lx] == 0))                 //Check for the continuity
                                        {
                                            belowx = lx;
                                            checkbelow = true;
                                            onceb = 0;
                                        }
                                    }
                                    else if((scn_prsnt[lx] == 0) && (pntr_flg[lx] == 0))             		//Present pixel is black and has not been processed
                                    {
                                        if((clr_flg[lx-1] == blob_count) || (clr_flg[lx+1] == blob_count))	//Check for the continuity with previous scanned data
                                        {
                                            cv_adjustBox(lx,ly,cornerA,cornerB);

                                            pntr_flg [lx] = 255;

                                            clr_flg [lx] = blob_count;

                                            n = n+1;

                                            if((onceb == 1) && (scn_next[lx] == 0))
                                            {
                                                belowx = lx;
                                                checkbelow = true;
                                                onceb = 0;
                                            }
                                        }
                                        else if((scn_prsnt[lx] == 0) && (clr_flg[lx-2] == blob_count))		// Check for the continuity with previous scanned data
                                        {
                                            cv_adjustBox(lx,ly,cornerA,cornerB);

                                            pntr_flg [lx] = 255;

                                            clr_flg [lx] = blob_count;

                                            n = n+1;

                                            if((onceb == 1) && (scn_next[lx] == 0))
                                            {
                                                belowx = lx;
                                                checkbelow = true;
                                                onceb = 0;
                                            }
                                        }
                                        // Check for the continuity with previous scanned data
                                        else if((scn_prsnt[lx] == 0) && ((clrprev_flg[lx-1] == blob_count) || (clrprev_flg[lx] == blob_count) || (clrprev_flg[lx+1] == blob_count)))
                                        {
                                            cv_adjustBox(lx,ly,cornerA,cornerB);

                                            pntr_flg [lx] = 255;

                                            clr_flg [lx] = blob_count;

                                            n = n+1;

                                            if((onceb == 1) && (scn_next[lx] == 0))
                                            {
                                                belowx = lx;
                                                checkbelow = true;
                                                onceb = 0;
                                            }

                                        }
                                        else
                                        {
                                            continue;
                                        }

                                    }
                                    else
                                    {
                                        clr_flg[lx] = 0;	// Current pixel is not a part of any blob
                                    }
                                }	// End of scanning of a particular row of the blob
                            }
                            else	// If there is no continuity of the blob in the next row break from blob scan loop
                            {
                                break;
                            }

                            for(int q = 0; q < thres->width; ++q)	// Blob numbers of current row becomes Blob number of previous row for the next iteration of "row scan" for this particular blob
                            {
                                clrprev_flg[q]= clr_flg[q];
                            }
                            ly++;
                        }
                        // End of the Blob scanning routine 


			// At this point after scanning image data, A blob (or 'connected component') is obtained. We use this Blob for further analysis to confirm it is a Marker.

			
			// Get the Rectangular extent of the blob. This is used to estimate the span of the blob
			// If it too small, say only few pixels, it is too good to be true that it is a Marker. Thus reducing erroneous decoding
			rectw = abs(cornerA.x - cornerB.x);
			recth = abs(cornerA.y - cornerB.y);
			aspect_ratio = (double)rectw / (double)recth;

                        if((n > min_blob_sze) && (n < max_blob_sze))		// Reduces chances of decoding erroneous 'Blobs' as markers
                        {
                            if((aspect_ratio > 0.33) && (aspect_ratio < 3.0))	// Increases chances of identified 'Blobs' to be close to Square 
                            {
                                // Good Blob; Mark it
								cvRectangle(img,cornerA,cornerB,CV_RGB(255,0,0),1);
                            }	
                            else	// Discard the blob data
                            {                      
                                blob_count = blob_count -1;	
                            }
                        }
                        else  		// Discard the blob data               
                        {
                            blob_count = blob_count -1;		
                        }

                }
                else     // If current pixel is not black do nothing
                {
                    continue;
                }
		}	// End full scan of the image by incrementing x
        }	// End full scan of the image by incrementing y
	

		cvShowImage("Camera",img);
		key = cvWaitKey(1);	// OPENCV: wait for 1ms before accessing next frame

	}	// End of 'while' loop

	cvDestroyWindow( "Camera" );	// Release various parameters

	cvReleaseImage(&img);
	cvReleaseImage(&gray);
	cvReleaseImage(&thres);
	cvReleaseImage(&prcs_flg);

    	return 0;
}
// End of Main Loop
//------------------------------------------------------------------------------------------------------------------------


// Routines used in Main loops

// Routine to update Bounding Box corners with farthest corners in that Box
void cv_adjustBox(int x, int y, CvPoint& A, CvPoint& B)
{
    if(x < A.x)
        A.x = x;

    if(y < A.y)
        A.y = y;

    if(x > B.x)
        B.x = x;

    if(y > B.y)
        B.y = y;
}

// EOF