#ifndef COLOR_MATCH_H_
#define COLOR_MATCH_H_

#include <cstdint>

// custom includes
#include "dlib_pixel_operations.h"

// dlib includes
#include <dlib/matrix.h>
#include <dlib/image_transforms.h>

// ----------------------------------------------------------------------------
dlib::matrix<uint32_t, 1, 2> get_color_match(dlib::matrix<dlib::rgb_pixel>& img, dlib::mmod_rect& det)
{
    uint64_t r, c;

    dlib::hsi_pixel red_ll1(0, 0, 64);
    dlib::hsi_pixel red_ul1(15, 255, 255);
    dlib::hsi_pixel red_ll2(240, 0, 64);
    dlib::hsi_pixel red_ul2(255, 255, 255);

    dlib::hsi_pixel blue_ll(155, 0, 64);
    dlib::hsi_pixel blue_ul(185, 255, 255);

    //dlib::hsi_pixel black_ll(0, 0, 0);
    //dlib::hsi_pixel black_ul(255, 64, 48);
    dlib::rgb_pixel black_ll(0, 0, 0);
    dlib::rgb_pixel black_ul(48, 48, 48);


    dlib::hsi_pixel gray_ll(0, 0, 48);
    dlib::hsi_pixel gray_ul(255, 255, 128);
    //dlib::rgb_pixel gray_ll(65, 65, 65);
    //dlib::rgb_pixel gray_ul(128, 128, 128);

    const int w = 20, h = 20;

    dlib::matrix<uint16_t> red_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> blue_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> black_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> gray_mask = dlib::zeros_matrix<uint16_t>(h, w);

    // crop out the detection
    dlib::point ctr = dlib::center(det.rect);

    dlib::matrix<dlib::rgb_pixel> rgb_crop = dlib::subm(img, dlib::centered_rect(ctr, w, h));
    dlib::matrix<dlib::hsi_pixel> hsi_crop;
    dlib::assign_image(hsi_crop, rgb_crop);

    dlib::hsi_pixel p;
    dlib::rgb_pixel q;

    for (r = 0; r < hsi_crop.nr(); ++r)
    {
        for (c = 0; c < hsi_crop.nc(); ++c)
        {
            dlib::assign_pixel(p, hsi_crop(r, c));
            dlib::assign_pixel(q, rgb_crop(r, c));

            // test for red backpack
            if ((p >= red_ll1) && (p <= red_ul1))
            {
                red_mask(r, c) = 1;
            }
            else if ((p >= red_ll2) && (p <= red_ul2))
            {
                red_mask(r, c) = 1;
            }
            else if ((p >= blue_ll) && (p <= blue_ul))
            {
                blue_mask(r, c) = 1;
            }
            else if ((q >= black_ll) && (q <= black_ul))
            {
                black_mask(r, c) = 1;
            }
            //else if ((p >= gray_ll) && (p <= gray_ul))
            //{
            //    gray_mask(r, c) = 1;
            //}

        }
    }

    dlib::matrix<uint32_t, 1, 2> res;
    
    uint32_t mask_sum = (uint32_t)dlib::sum(red_mask) + (uint32_t)dlib::sum(blue_mask) + (uint32_t)dlib::sum(black_mask);
    //res = (uint32_t)dlib::sum(red_mask), (uint32_t)dlib::sum(blue_mask), (uint32_t)dlib::sum(black_mask), (uint32_t)dlib::sum(gray_mask);
    res = mask_sum, (uint32_t)(rgb_crop.size() - mask_sum);
    return res;

}   // end of get_color_match



#endif  //COLOR_MATCH_H_