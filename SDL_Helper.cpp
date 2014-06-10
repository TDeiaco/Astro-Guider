//These functions are to be incorperated into the engine class

#include <SDL/SDL.h>
#include <iostream>

using namespace std;

SDL_Color IndexToColor(int index, SDL_PixelFormat *fmt)
{
  SDL_Color color;
  SDL_GetRGB(index, fmt, &color.r, &color.g, &color.b);
  return color;
}

void FPSTest(bool capped)
{
  static unsigned long fps;
  static long counter;
  char buf[50];

  counter++;
  if(fps + 1000 < SDL_GetTicks())
    {
      //	if(capped)
      //		sprintf(buf,"Capped at %d FPS",counter); 
      //	else
      //		sprintf(buf,"%d FPS",counter);
      //SDL_WM_SetCaption(buf,0);       //printf("FPS: %d", counter);
      fps = SDL_GetTicks();
      counter = 0;
    }
}

unsigned long ReturnFPS()
{
  static unsigned long fps;
  static long counter;
  static unsigned long rtn = 0;

  counter++;
  if(fps + 1000 < SDL_GetTicks())
    {
      rtn = counter;
      fps = SDL_GetTicks();
      counter = 0;
    }
  return rtn;
}

SDL_Surface* CreateEmptySurface(int w, int h, bool fill, SDL_Color color)
{
  SDL_Surface *temp = 0;
  //Spcift the color masks
#if SDL_BYTEORDER == SDL_BIG_ENDIAN
  int rmask = 0xff000000;
  int gmask = 0x00ff0000;
  int bmask = 0x0000ff00;
  int amask = 0x000000ff;
#else
  int rmask = 0x000000ff;
  int gmask = 0x0000ff00;
  int bmask = 0x00ff0000;
  int amask = 0xff000000;
#endif
	
  temp = SDL_CreateRGBSurface(SDL_SWSURFACE, w,h, 32, rmask,gmask,bmask,amask);
  if(temp == NULL){
    cout<<"SDL_CreateRGBSurface failed!"<<endl;
    cout<<"Error: "<<SDL_GetError<<endl;
  }else {
    if(fill)
      SDL_FillRect(temp, 0, SDL_MapRGB(temp->format, color.r, color.g, color.b));
  }
  return temp;
}

void SaveScreenShot(const char *name, int count, SDL_Surface *g_screen)
{
  char buffer[256];
  //sprintf(buffer, "%s%d.bmp", name, count);
  SDL_SaveBMP(g_screen, "Screenshot");
}

void SetPixel(SDL_Surface *surface, int x, int y, Uint32 pixel)
{
  int bpp = surface->format->BytesPerPixel;
  /* Here p is the address to the pixel we want to set */
  Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

  switch(bpp) 
    {
    case 1:
      *p = pixel;
      break;

    case 2:
      *(Uint16 *)p = pixel;
      break;

    case 3:
      if(SDL_BYTEORDER == SDL_BIG_ENDIAN) {
	p[0] = (pixel >> 16) & 0xff;
	p[1] = (pixel >> 8) & 0xff;
	p[2] = pixel & 0xff;
      } else {
	p[0] = pixel & 0xff;
	p[1] = (pixel >> 8) & 0xff;
	p[2] = (pixel >> 16) & 0xff;
      }
      break;

    case 4:
      *(Uint32 *)p = pixel;
      break;
    }
}

Uint32 GetPixel(SDL_Surface *surface, int x, int y)
{
  int bpp = surface->format->BytesPerPixel;
  //printf("Bytes per Pixel : %d \n", bpp);
  /* Here p is the address to the pixel we want to set */
  Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;


  /* if(SDL_BYTEORDER == SDL_BIG_ENDIAN) {
     p[0] = (pixel >> 16) & 0xff;
     p[1] = (pixel >> 8) & 0xff;
     p[2] = pixel & 0xff;
     } else {
     p[0] = pixel & 0xff;
     p[1] = (pixel >> 8) & 0xff;
     p[2] = (pixel >> 16) & 0xff;
     }
  */

  return SDL_MapRGB(surface->format, p[0], p[1], p[2]);

}




