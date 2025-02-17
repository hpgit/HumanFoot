import OpenGL.GL as gl

'''
   Write the current view to a file
   The multiple fputc()s can be replaced with
      fwrite(image,width*height*3,1,fptr);
   If the memory pixel order is the same as the destination file format.
'''
def windowDump():
    int i,j;
    FILE *fptr;
    static int counter = 0; # This supports animation sequences
    char fname[32];
    unsigned char *image;

    #Allocate our buffer for the image */
    '''
    if ((image = malloc(3*width*height*sizeof(char))) == NULL):
       fprintf(stderr,"Failed to allocate memory for image\n")
       return(FALSE)
    '''

    gl.glPixelStorei(gl.GL_PACK_ALIGNMENT,1);

    # Open the file
    '''
    if (stereo):
       sprintf(fname,"L_%04d.raw",counter)
    else:
       sprintf(fname,"C_%04d.raw",counter)
    if ((fptr = fopen(fname,"w")) == NULL):
       fprintf(stderr,"Failed to open file for window dump\n")
       return(FALSE)
    '''

    #/* Copy the image into our buffer */
    gl.glReadBuffer(gl.GL_BACK_LEFT)
    gl.glReadPixels(0,0,width,height,gl.GL_RGB,gl.GL_UNSIGNED_BYTE,image);

    #/* Write the raw file */
    #/* fprintf(fptr,"P6\n%d %d\n255\n",width,height); for ppm */
    for (j=height-1;j>=0;j--):
       for (i=0;i<width;i++):
          fputc(image[3*j*width+3*i+0],fptr);
          fputc(image[3*j*width+3*i+1],fptr);
          fputc(image[3*j*width+3*i+2],fptr);

    fclose(fptr)

    if (stereo):
       # Open the file
       sprintf(fname,"R_%04d.raw",counter);
       if ((fptr = fopen(fname,"w")) == NULL):
          fprintf(stderr,"Failed to open file for window dump\n")
          return(FALSE)

       # Copy the image into our buffer */
       gl.glReadBuffer(gl.GL_BACK_RIGHT)
       gl.glReadPixels(0,0,width,height,gl.GL_RGB,gl.GL_UNSIGNED_BYTE,image)

       # Write the raw file */
       # fprintf(fptr,"P6\n%d %d\n255\n",width,height); for ppm */
       for (j=height-1;j>=0;j--):
         for (i=0;i<width;i++):
            fputc(image[3*j*width+3*i+0],fptr);
            fputc(image[3*j*width+3*i+1],fptr);
            fputc(image[3*j*width+3*i+2],fptr);

      fclose(fptr)

   # Clean up
   counter+=1
   free(image)
   return (TRUE)
