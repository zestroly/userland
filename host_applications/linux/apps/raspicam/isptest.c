#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include <sysexits.h>

#define VERSION_STRING "v1.3.8"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

int main(int argc, char *argv[])
{
	int src_w, src_h, dst_w, dst_h;
	MMAL_FOURCC_T src_enc, dst_enc;
	MMAL_COMPONENT_T *source=NULL, *isp=NULL, *render=NULL;
	MMAL_CONNECTION_T *src_isp=NULL, *isp_render=NULL;
	MMAL_STATUS_T status;
	MMAL_ES_FORMAT_T *format;
	int ret = 1;

	if(argc < 7)
	{
		printf("Usage: %s <src w> <src h> <src encoding> <dst w> <dst h> <dst encoding> [disp rect x,y,w,h]\n", argv[0]);
		exit(1);
	}

	src_w = atoi(argv[1]);
	src_h = atoi(argv[2]);
	src_enc = *(MMAL_FOURCC_T*)argv[3];

	dst_w = atoi(argv[4]);
	dst_h = atoi(argv[5]);
	dst_enc = *(MMAL_FOURCC_T*)argv[6];

	status = mmal_component_create("vc.ril.source", &source);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create source");
		goto destroy;
	}

	status = mmal_component_create("vc.ril.isp", &isp);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create isp");
		goto destroy;
	}

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &render);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create render");
		goto destroy;
	}

	mmal_component_enable(source);
	mmal_component_enable(isp);
	mmal_component_enable(render);

	if(argc > 7)
	{
		MMAL_DISPLAYREGION_T param;
		int tmp;
		param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
		param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);

		tmp = sscanf(argv[7], "%d,%d,%d,%d",
			&param.dest_rect.x, &param.dest_rect.y,
			&param.dest_rect.width, &param.dest_rect.height);
		if (tmp == 4)
		{
			param.set |= (MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_FULLSCREEN);
			param.fullscreen = 0;
		}
		else
		{
			param.set |= MMAL_DISPLAY_SET_FULLSCREEN;
			param.fullscreen = 1;
		}

		status = mmal_port_parameter_set(render->input[0], &param.hdr);
	}

	format = source->output[0]->format;
	format->encoding = src_enc;
	format->es->video.width = VCOS_ALIGN_UP(src_w, 32);
	format->es->video.height = VCOS_ALIGN_UP(src_h, 16);
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = src_w;
	format->es->video.crop.height = src_h;

	status = mmal_port_format_commit(source->output[0]);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("source format commit failed - %s", mmal_status_to_string(status));
		goto destroy;
	}

	status = mmal_port_parameter_set_int32(source->output[0], MMAL_PARAMETER_VIDEO_SOURCE_PATTERN, MMAL_VIDEO_SOURCE_PATTERN_BLOCKS);
	if(status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set source pattern");
		goto destroy;
	}

	if(dst_enc == 'xxxx')
	{
		//Bypass mode
		vcos_log_error("BYPASS MODE");
		status =  mmal_connection_create(&src_isp, source->output[0], render->input[0], MMAL_CONNECTION_FLAG_TUNNELLING);
		if (status == MMAL_SUCCESS)
		{
			status =  mmal_connection_enable(src_isp);
			if (status != MMAL_SUCCESS)
				goto destroy;
		}
	}
	else
	{
		status =  mmal_connection_create(&src_isp, source->output[0], isp->input[0], MMAL_CONNECTION_FLAG_TUNNELLING);
		if (status == MMAL_SUCCESS)
		{
			status =  mmal_connection_enable(src_isp);
			if (status != MMAL_SUCCESS)
				goto destroy;
		}

		format = isp->output[0]->format;
		format->encoding = dst_enc;
		format->es->video.width = VCOS_ALIGN_UP(dst_w, 32);
		format->es->video.height = VCOS_ALIGN_UP(dst_h, 16);
		format->es->video.crop.x = 0;
		format->es->video.crop.y = 0;
		format->es->video.crop.width = dst_w;
		format->es->video.crop.height = dst_h;

		status = mmal_port_format_commit(isp->output[0]);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("isp format commit failed - %s", mmal_status_to_string(status));
			goto destroy;
		}

		status =  mmal_connection_create(&isp_render, isp->output[0], render->input[0], MMAL_CONNECTION_FLAG_TUNNELLING);
		if (status == MMAL_SUCCESS)
		{
			status =  mmal_connection_enable(isp_render);
			if (status != MMAL_SUCCESS)
				goto destroy;
		}
	}

	sleep(5);
	ret = 0;

destroy:
	if(src_isp)
	{
		mmal_connection_disable(src_isp);
		mmal_connection_destroy(src_isp);
	}
	if(isp_render)
	{
		mmal_connection_disable(isp_render);
		mmal_connection_destroy(isp_render);
	}
	if(render)
		mmal_component_destroy(render);
	if(isp)
		mmal_component_destroy(isp);
	if(source)
		mmal_component_destroy(source);

	return ret;
}