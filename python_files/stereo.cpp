void CameraCalibration::setUndistStereoMap(const cv::Mat &R, const cv::Mat &P, const cv::Rect &roi) 
{
    std::cout << "\n\nComputing the stereo rectification mapping!\n";
    
    // CV_16SC2 = 11 / CV_32FC1 = 5
    if( model_ == Pinhole )
    {
        cv::initUndistortRectifyMap(Kcv_, Dcv_, R, P, img_size_, 11, undist_map_x_, undist_map_y_);
    }
    else if ( model_ == Fisheye )
    {
        cv::fisheye::initUndistortRectifyMap(Kcv_, Dcv_, R, P, img_size_, 11, undist_map_x_, undist_map_y_);        
    }

    cv::cv2eigen(R,Rrectraw_);
    
    Eigen::Matrix4d Pe;
    cv::cv2eigen(P,Pe);z

    fx_ = Pe(0,0);
    fy_ = Pe(1,1);
    cx_ = Pe(0,2);
    cy_ = Pe(1,2);

    K_ = Pe.block<3,3>(0,0);

    cv::eigen2cv(K_, Kcv_);
    Dcv_.release();
    D_.setZero();

    k1_ = 0.; k2_ = 0.;
    p1_ = 0.; p2_ = 0.;
    
    iK_ = K_.inverse();
    ifx_ = iK_(0,0);
    ify_ = iK_(1,1);
    icx_ = iK_(0,2);
    icy_ = iK_(1,2);

    Tc0ci_ = Sophus::SE3d();
    Tc0ci_.translation() = Eigen::Vector3d(-1. * Pe(0,3) / fx_,
                                           -1. * Pe(1,3) / fx_,
                                           -1. * Pe(2,3) / fx_);

    Tcic0_ = Tc0ci_.inverse();

    cv::eigen2cv(Tc0ci_.rotationMatrix(), Rcv_c0ci_);
    cv::eigen2cv(Tc0ci_.translation(), tcv_c0ci_);

    Rcv_cic0_ = Rcv_c0ci_.clone();
    tcv_cic0_ = -1. * tcv_c0ci_.clone();

    // Setup roi mask / rect
    setROIMask(roi);

    std::cout << "\n Undist+Rect Camera Calibration set as : \n\n";
    std::cout << "\n K = \n" << K_;
    std::cout << "\n\n D = " << D_.transpose();
    std::cout << "\n\n ROI = " << roi_rect_;
}

        cv::stereoRectify(
                pcalib_model_left_->Kcv_, pcalib_model_left_->Dcv_,
                pcalib_model_right_->Kcv_, pcalib_model_right_->Dcv_,
                pcalib_model_left_->img_size_, 
                pcalib_model_right_->Rcv_cic0_, 
                pcalib_model_right_->tcv_cic0_,
                Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY, 
                pslamstate_->alpha_,
                pcalib_model_left_->img_size_, 
                &rectleft, &rectright
                );


        pcalib_model_right_.reset( 
                    new CameraCalibration(
                            pslamstate_->cam_right_model_, 
                            pslamstate_->fxr_, pslamstate_->fyr_, 
                            pslamstate_->cxr_, pslamstate_->cyr_,
                            pslamstate_->k1r_, pslamstate_->k2r_, 
                            pslamstate_->p1r_, pslamstate_->p2r_,
                            pslamstate_->img_right_w_, 
                            pslamstate_->img_right_h_
                            ) 
                        );
        
        // TODO: Change this and directly add the extrinsic parameters within the 
        // constructor (maybe set default parameters on extrinsic with identity / zero)
        pcalib_model_right_->setupExtrinsic(pslamstate_->T_left_right_);