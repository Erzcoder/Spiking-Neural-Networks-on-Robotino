# -*- coding: utf-8 -*-
"""
Created on Sat Feb 7 15:57:23 2017

@author: elie aljalbout
@email : elie.el.jalbout2@gmail.com

The aim of this class if to convert a binary thresholded image into a set of int64 variables, and the reverse :
to group multiple int64 variables coming from the same image (by checking the ids) into one complete image.

Each int64 variable is represented like following:
    [sign=0] [pixels] [ id ]
    [   1  ] [  60  ] [ 3  ]
    
    
The way it works is simple:
Consider you have the following binary image:   11111010101010101010101010101010101010101010101010101010101010100 
In the encoding function you'll split this to 2 int64 elements: 60+5=64 (size of the above image) like following:
    111110101010101010101010101010101010101010101010101010101010
    101000000000000000000000000000000000000000000000000000000000
    
The second int64 is mainly 10100 and the rest is a 0 padding which is then removed in the decoding phase

In the decoding phase you give the decode function an int64 element, it will return an image array in case 
you've called enough times with elements having same ids to form an image of the size DEFAULTIMAGESIZE
otherwise it'll retrun None signaling that no image is yet ready

How to use it:
    all methods are obviously static, which means there's no need to create an instance of it, just set the
    parameter DEFAULTIMAGESIZE to a value corresponding to your application case, then call the functions
    decodeInt64ToIntImage and encodeImageToInt64 correspondly
    
Quick test? 
Uncomment the comment area "Quick test"
"""

import numpy as np


class ImageToInt64Encoder:

    #configuration related parameters ====================
    DEFAULTIMAGESIZE = 65       #pixels
    OFFSET           = 60       #number of bits in an int64 variable that contains pixel from threshold(binary) image
    IDLEN            = 3        # number of bits representing the id of an image
    #=====================================================
    
    theId          = 3        #used for generating ids
    lastId         = -1       #last received id
    currentChunk   = '' 
    chunkCounter   = 0        #number of grouped int64 variables
    
    #generates and id that fits in IDLEN bits
    @staticmethod
    def IdGenerator():
        ImageToInt64Encoder.theId=(ImageToInt64Encoder.theId+1)%(2**ImageToInt64Encoder.IDLEN)
        return ImageToInt64Encoder.theId
    
    #calculate the number of int64 variables needed for an image and the offset of the last one
    @staticmethod
    def calculateConfParams(length):
        return int(length/ImageToInt64Encoder.OFFSET)+1,length%ImageToInt64Encoder.OFFSET

    #split int64 variable to offset and id
    @staticmethod
    def getImageChunkAndId(toDecode):
        toDecode = bin(int(toDecode)) #convert to string of binary
        toDecode = toDecode[2:]

        theId  = toDecode[(len(toDecode)-ImageToInt64Encoder.IDLEN):]
        offset = toDecode[:(len(toDecode)-ImageToInt64Encoder.IDLEN)]
        return theId,offset

    #add 0s padding to int64 elems representing a small number of pixels<OFFSET (usually last element of image)
    @staticmethod
    def addPadding(binArr):
        binStr=binArr[len(binArr)-1]
        paddNum=ImageToInt64Encoder.OFFSET-len(binStr)
        for _ in range(paddNum):
            binStr+='0'          #sign bit always 0 for less confusion
        binArr[len(binArr)-1]=binStr

    #add the id to the binary string
    @staticmethod        
    def addIds(binArr,theId):
        toAdd = ImageToInt64Encoder.IDLEN-len(str(bin(theId)))
        did   = ''
        for _ in range(toAdd):
            did+='0'
        theId=did+str(bin(theId))
            
        for ind in range(len(binArr)):
            binArr[ind]+=str(theId[2:])
    
            
    @staticmethod        
    def convertBinArrToInt64Array(binArr):
        spnkrInput=[]
        for binStr in binArr:
            bini   = int(binStr,2)
            inputz = np.int64()
            inputz = bini
            spnkrInput.append(inputz)
        return spnkrInput
    
    @staticmethod    
    def chunkToImageArray(chunk,numInts,lastImageOffset):
        print((numInts-1)*ImageToInt64Encoder.OFFSET+lastImageOffset)
        chunk=chunk[:(numInts-1)*ImageToInt64Encoder.OFFSET+lastImageOffset]
        ImageToInt64Encoder.currentChunk=''
        chunk=list(map(int,list(chunk)))
        
        return chunk
        
    
    #Turns an image into an array of int64 variables
    @staticmethod
    def encodeImageToInt64(imgArr):
        
        newId=ImageToInt64Encoder.IdGenerator()
        numInts,lastImageOffset=ImageToInt64Encoder.calculateConfParams(ImageToInt64Encoder.DEFAULTIMAGESIZE)
        binArr=[]
        for var in range(numInts):
            strBin='0'
            prange=ImageToInt64Encoder.OFFSET if var<numInts-1 else lastImageOffset
            for pix in range(prange):
                strBin+=str(imgArr[var*ImageToInt64Encoder.OFFSET+pix])
            binArr.append(strBin)
        ImageToInt64Encoder.addPadding(binArr)
        ImageToInt64Encoder.addIds(binArr,newId)
        #print(binArr)
        return ImageToInt64Encoder.convertBinArrToInt64Array(binArr)
        
        
    @staticmethod
    def decodeInt64ToIntImage(toDecode):
        
        imageId,offset=ImageToInt64Encoder.getImageChunkAndId(toDecode)
        
        numInts,lastImageOffset=ImageToInt64Encoder.calculateConfParams(ImageToInt64Encoder.DEFAULTIMAGESIZE)
        
        if imageId!=ImageToInt64Encoder.lastId:
            ImageToInt64Encoder.lastId          = imageId
            ImageToInt64Encoder.currentChunk    = offset
            ImageToInt64Encoder.chunkCounter    = 1
            return None
        
        ImageToInt64Encoder.currentChunk+=offset
        ImageToInt64Encoder.chunkCounter+=1
        if ImageToInt64Encoder.chunkCounter==numInts:
            ImageToInt64Encoder.chunkCounter    = 0
            ImageToInt64Encoder.lastId          = -1
            return ImageToInt64Encoder.chunkToImageArray(ImageToInt64Encoder.currentChunk,numInts,lastImageOffset)
            
        ImageToInt64Encoder.lastId=imageId
    
 
"""Quick test

imgArr=[1,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1]
test=ImageToInt64Encoder.encodeImageToInt64(imgArr)
print(test)
print(ImageToInt64Encoder.decodeInt64ToIntImage(test[0]))
print(ImageToInt64Encoder.decodeInt64ToIntImage(test[1]))

"""
