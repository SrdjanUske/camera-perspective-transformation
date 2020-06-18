#ifndef __PERSPECTIVE_TRANSFORM_H__
#define __PERSPECTIVE_TRANSFORM_H__

#ifdef __cplusplus
extern "C" {
#endif

extern "C" bool mouse_click_and_param_init(void* init_bgr_frame);
extern "C" void get_perspective_transform(void* input_bgr_frame);

#ifdef __cplusplus
}
#endif
#endif /*__PERSPECTIVE_TRANSFORM_H__*/