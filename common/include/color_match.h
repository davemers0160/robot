#ifndef COLOR_MATCH_H_
#define COLOR_MATCH_H_

#include <cstdint>

// custom includes
#include "dlib_pixel_operations.h"

// dlib includes
#include <dlib/matrix.h>
#include <dlib/image_transforms.h>

// ----------------------------------------------------------------------------
// In the HSI colorspace represents every color with three components: hue ( H ), saturation ( S ), intensity ( I ).
// The Hue component describes the color itself in the form of an angle between [0,360] degrees. 
// - Red:       0 degrees
// - Yellow:   60 degrees
// - Green:   120 degrees
// - Cyan:    180 degrees
// - Blue:    240 degrees
// - Magenta: 300 degrees
// in the dlib library the H component is scaled by 255/360
// The Saturation component signals how much the color is polluted with white color. The range of the S component is [0,255]
// The Intensity range is between [0,255] and 0 means black, 255 means white.

dlib::matrix<uint32_t, 1, 2> get_color_match(dlib::matrix<dlib::rgb_pixel>& img, dlib::mmod_rect& det)
{
    uint64_t r, c;
    const int w = 20, h = 20;

    // define the HSI boundaries for each color to test
    dlib::hsi_pixel red_ll1(0, 0, 30);
    dlib::hsi_pixel red_ul1(20, 255, 204);
    dlib::hsi_pixel red_ll2(235, 0, 30);
    dlib::hsi_pixel red_ul2(255, 255, 204);

    dlib::hsi_pixel blue_ll(150, 0, 30);
    dlib::hsi_pixel blue_ul(190, 255, 204);

    dlib::hsi_pixel green_ll(65, 0, 30);
    dlib::hsi_pixel green_ul(105, 255, 204);

    //dlib::hsi_pixel black_ll(0, 0, 0);
    //dlib::hsi_pixel black_ul(255, 64, 48);
    dlib::rgb_pixel black_ll(0, 0, 0);
    dlib::rgb_pixel black_ul(48, 48, 48);

    // create the masks
    dlib::matrix<uint16_t> red_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> blue_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> black_mask = dlib::zeros_matrix<uint16_t>(h, w);
    dlib::matrix<uint16_t> green_mask = dlib::zeros_matrix<uint16_t>(h, w);
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
            else if ((p >= green_ll) && (p <= green_ul))
            {
                green_mask(r, c) = 1;
            }
            else if ((q >= black_ll) && (q <= black_ul))
            {
                black_mask(r, c) = 1;
            }

        }
    }

    dlib::matrix<uint32_t, 1, 2> res;
    
    uint32_t mask_sum = (uint32_t)dlib::sum(red_mask) + (uint32_t)dlib::sum(blue_mask) + (uint32_t)dlib::sum(black_mask);

    res = mask_sum, (uint32_t)(rgb_crop.size() - mask_sum);
    return res;

}   // end of get_color_match

// ----------------------------------------------------------------------------

#endif  //COLOR_MATCH_H_