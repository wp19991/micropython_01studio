#ifndef __GLOBAL_H
#define __GLOBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include "systick.h"
#include "pendsv.h"


#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"


extern mp_obj_t get_path(const char *src_path , uint8_t *res );
extern mp_obj_t file_type(const char *fileName);
extern FRESULT f_open_helper(FIL *fp, const TCHAR *path, BYTE mode);

#ifdef __cplusplus
}
#endif
#endif 


#ifdef __GLOBAL_H
//-----------------------------------------------------------------------
 __weak char * strrchr(const char * str,int ch)
 {
		char *p = (char *)str;
		while (*str) str++;
		while (str-- != p && *str != (char)ch);
		if (*str == (char)ch)
				 return( (char *)str );
		return(NULL);
 }

 //-----------------------------------------------------------------------
 //路径解析出来
 //返回0 为flash 1为sd。否则解析失败。
 //src_path:输入路径 ret_path：返回的相对路径
 __weak mp_obj_t get_path(const char *src_path , uint8_t *res ) {
 
	 uint8_t date_len = 0 ,cp_len =0;
 
	 char upda_str[50];
 
	 char cp_str[7];
	 char *ret_path;
 
	 memset(upda_str, '\0', sizeof(upda_str));
	 strncpy(upda_str,&src_path[1] , strlen(src_path)-1); 
	 date_len = strlen(upda_str);

	 ret_path=strchr(upda_str,'/');
	 if(ret_path == 0)	mp_raise_ValueError(MP_ERROR_TEXT("file path error"));
	 
	 cp_len = date_len - strlen(ret_path);
 
	 memset(cp_str, '\0', 7);
	 strncpy(cp_str,upda_str,cp_len);

 		*res = 2;
	 if(strncmp(cp_str , "flash" , 5) == 0) 	 *res = 0; 
	 else if(strncmp(cp_str , "sd" , 2) == 0)  *res = 1;
	 else	
	 	{
			mp_raise_ValueError(MP_ERROR_TEXT("no find sd or flash path"));
		}
 
	 date_len = (date_len - cp_len - 1);
	 memset(upda_str, '\0', sizeof(upda_str));
	 strncpy(upda_str,&src_path[cp_len+2] , date_len); 
 
	 return mp_obj_new_str(upda_str, date_len);
 }

 __weak mp_obj_t file_type(const char *fileName)
{
	char dest[10];
	memset(dest, '\0', sizeof(dest));
	char *ret = strrchr(fileName , '.');
	if(ret == NULL){
		mp_raise_TypeError(MP_ERROR_TEXT("no find file type"));
	}
	strncpy(dest,&ret[1] , strlen(ret)-1); 
	return mp_obj_new_str(dest, strlen(ret)-1);
}
 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 STATIC FATFS *lookup_path(const TCHAR **path) {
		 mp_vfs_mount_t *fs = mp_vfs_lookup_path(*path, path);
		 if (fs == MP_VFS_NONE || fs == MP_VFS_ROOT) {
		 
				 return NULL;
		 }
		 // here we assume that the mounted device is FATFS
		 return &((fs_user_mount_t*)MP_OBJ_TO_PTR(fs->obj))->fatfs;
 }
 
  __weak FRESULT f_open_helper(FIL *fp, const TCHAR *path, BYTE mode) {
		 FATFS *fs = lookup_path(&path);
		 if (fs == NULL) {
				 return FR_NO_PATH;
		 }
		 return f_open(fs, fp, path, mode);
 }
 //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#endif
