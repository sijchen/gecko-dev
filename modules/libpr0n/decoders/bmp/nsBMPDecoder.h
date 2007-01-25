/* vim:set tw=80 expandtab softtabstop=4 ts=4 sw=4: */
/* ***** BEGIN LICENSE BLOCK *****
 * Version: MPL 1.1/GPL 2.0/LGPL 2.1
 *
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
 * for the specific language governing rights and limitations under the
 * License.
 *
 * The Original Code is the Mozilla BMP Decoder.
 *
 * The Initial Developer of the Original Code is
 * Christian Biesinger <cbiesinger@web.de>.
 * Portions created by the Initial Developer are Copyright (C) 2001
 * the Initial Developer. All Rights Reserved.
 *
 * Contributor(s):
 *
 * Alternatively, the contents of this file may be used under the terms of
 * either the GNU General Public License Version 2 or later (the "GPL"), or
 * the GNU Lesser General Public License Version 2.1 or later (the "LGPL"),
 * in which case the provisions of the GPL or the LGPL are applicable instead
 * of those above. If you wish to allow use of your version of this file only
 * under the terms of either the GPL or the LGPL, and not to allow others to
 * use your version of this file under the terms of the MPL, indicate your
 * decision by deleting the provisions above and replace them with the notice
 * and other provisions required by the GPL or the LGPL. If you do not delete
 * the provisions above, a recipient may use your version of this file under
 * the terms of any one of the MPL, the GPL or the LGPL.
 *
 * ***** END LICENSE BLOCK ***** */


#ifndef _nsBMPDecoder_h
#define _nsBMPDecoder_h

#include "nsCOMPtr.h"
#include "imgIDecoder.h"
#include "imgIContainer.h"
#include "imgIDecoderObserver.h"
#include "gfxIImageFrame.h"

#define NS_BMPDECODER_CID \
{ /* {78c61626-4d1f-4843-9364-4652d98ff6e1} */ \
  0x78c61626, \
  0x4d1f, \
  0x4843, \
  { 0x93, 0x64, 0x46, 0x52, 0xd9, 0x8f, 0xf6, 0xe1 } \
}

struct BMPFILEHEADER {
    char signature[2]; // String "BM"
    PRUint32 filesize;
    PRInt32 reserved; // Zero
    PRUint32 dataoffset; // Offset to raster data

    PRUint32 bihsize;
};
#define BFH_LENGTH 18 // Note: For our purposes, we include bihsize in the BFH

#define OS2_BIH_LENGTH 12 // This is the real BIH size (as contained in the bihsize field of BMPFILEHEADER)
#define OS2_HEADER_LENGTH (BFH_LENGTH + 8)
#define WIN_HEADER_LENGTH (BFH_LENGTH + 36)

struct BMPINFOHEADER {
    PRInt32 width; // Uint16 in OS/2 BMPs
    PRInt32 height; // Uint16 in OS/2 BMPs
    PRUint16 planes; // =1
    PRUint16 bpp; // Bits per pixel.
    // The rest of the header is not available in OS/2 BMP Files
    PRUint32 compression; // 0=no compression 1=8bit RLE 2=4bit RLE
    PRUint32 image_size; // (compressed) image size. Can be 0 if compression==0
    PRUint32 xppm; // Pixels per meter, horizontal
    PRUint32 yppm; // Pixels per meter, vertical
    PRUint32 colors; // Used Colors
    PRUint32 important_colors; // Number of important colors. 0=all
};

struct colorTable {
    PRUint8 red;
    PRUint8 green;
    PRUint8 blue;
};

struct bitFields {
    PRUint32 red;
    PRUint32 green;
    PRUint32 blue;
    PRUint8 redLeftShift;
    PRUint8 redRightShift;
    PRUint8 greenLeftShift;
    PRUint8 greenRightShift;
    PRUint8 blueLeftShift;
    PRUint8 blueRightShift;
};

#define BITFIELD_LENGTH 12 // Length of the bitfields structure in the bmp file

#if defined WORDS_BIGENDIAN || defined IS_BIG_ENDIAN
// We must ensure that the entity is unsigned
// otherwise, if it is signed/negative, the MSB will be
// propagated when we shift
#define LITTLE_TO_NATIVE16(x) (((((PRUint16) x) & 0xFF) << 8) | \
                               (((PRUint16) x) >> 8))
#define LITTLE_TO_NATIVE32(x) (((((PRUint32) x) & 0xFF) << 24) | \
                               (((((PRUint32) x) >> 8) & 0xFF) << 16) | \
                               (((((PRUint32) x) >> 16) & 0xFF) << 8) | \
                               (((PRUint32) x) >> 24))
#else
#define LITTLE_TO_NATIVE16(x) x
#define LITTLE_TO_NATIVE32(x) x
#endif

#if !defined(MOZ_CAIRO_GFX) && (defined(XP_WIN) || defined(XP_OS2) || defined(XP_BEOS) || defined(MOZ_WIDGET_PHOTON))
#define BMP_GFXFORMAT gfxIFormats::BGR
#define RLE_GFXFORMAT_ALPHA gfxIFormats::BGR_A1
#else
#define USE_RGB
#define BMP_GFXFORMAT gfxIFormats::RGB
#define RLE_GFXFORMAT_ALPHA gfxIFormats::RGB_A1
#endif

#if defined(MOZ_CAIRO_GFX) || defined(XP_MAC) || defined(XP_MACOSX)
#define GFXBYTESPERPIXEL 4
#else
#define GFXBYTESPERPIXEL 3
#endif

// BMPINFOHEADER.compression defines
#define BI_RLE8 1
#define BI_RLE4 2
#define BI_BITFIELDS 3

// RLE Escape codes
#define RLE_ESCAPE       0
#define RLE_ESCAPE_EOL   0
#define RLE_ESCAPE_EOF   1
#define RLE_ESCAPE_DELTA 2

/// enums for mState
enum ERLEState {
    eRLEStateInitial,
    eRLEStateNeedSecondEscapeByte,
    eRLEStateNeedXDelta,
    eRLEStateNeedYDelta,    ///< mStateData will hold x delta
    eRLEStateAbsoluteMode,  ///< mStateData will hold count of existing data to read
    eRLEStateAbsoluteModePadded ///< As above, but another byte of data has to be read as padding
};

/**
 * Decoder for BMP-Files, as used by Windows and OS/2
 */
class nsBMPDecoder : public imgIDecoder
{
public:
    NS_DECL_ISUPPORTS
    NS_DECL_IMGIDECODER
    
    nsBMPDecoder();
    ~nsBMPDecoder();

private:
    /** Callback for ReadSegments to avoid copying the data */
    static NS_METHOD ReadSegCb(nsIInputStream* aIn, void* aClosure,
                               const char* aFromRawSegment, PRUint32 aToOffset,
                               PRUint32 aCount, PRUint32 *aWriteCount);

    /** Processes the data.
     * @param aBuffer Data to process.
     * @oaram count Number of bytes in mBuffer */
    NS_METHOD ProcessData(const char* aBuffer, PRUint32 aCount);

    /** Calculates the red-, green- and blueshift in mBitFields using
     * the bitmasks from mBitFields */
    NS_METHOD CalcBitShift();

    /** Sets the image data. mCurLine is used to get the row,
     * mDecoded is used to get the data */
    nsresult SetData();

    /** Sets the rle data. mCurLine is used to get the row,
     * mDecoded is used to get the data for the first row,
     * all subsequent rows are blank.
     * @param rows Number of rows of data to set */
    nsresult WriteRLERows(PRUint32 rows);

    nsCOMPtr<imgIDecoderObserver> mObserver;

    nsCOMPtr<imgIContainer> mImage;
    nsCOMPtr<gfxIImageFrame> mFrame;

    PRUint32 mPos;

    BMPFILEHEADER mBFH;
    BMPINFOHEADER mBIH;
    char mRawBuf[36];

    PRUint32 mLOH; ///< Length of the header

    PRUint32 mNumColors; ///< The number of used colors, i.e. the number of entries in mColors
    colorTable *mColors;

    bitFields mBitFields;

    PRUint8 *mRow;      ///< Holds one raw line of the image
    PRUint32 mRowBytes; ///< How many bytes of the row were already received
    PRInt32 mCurLine;   ///< Index of the line of the image that's currently being decoded
    PRUint8 *mAlpha;    ///< Holds one line of unpacked alpha data
    PRUint8 *mAlphaPtr; ///< Pointer within unpacked alpha data
    PRUint8 *mDecoded;  ///< Holds one line of color image data
    PRUint8 *mDecoding; ///< Pointer within image data

    ERLEState mState;   ///< Maintains the current state of the RLE decoding
    PRUint32 mStateData;///< Decoding information that is needed depending on mState

    PRUint32 mBpr;      ///< Cached image bytes per row

    /** Set mBFH from the raw data in mRawBuf, converting from little-endian
     * data to native data as necessary */
    void ProcessFileHeader();
    /** Set mBIH from the raw data in mRawBuf, converting from little-endian
     * data to native data as necessary */
    void ProcessInfoHeader();
};

/** Sets the pixel data in aDecoded to the given values.
 * The variable passed in as aDecoded will be moved on 3 or 4 bytes! */
inline void SetPixel(PRUint8*& aDecoded, PRUint8 aRed, PRUint8 aGreen, PRUint8 aBlue, PRUint8 aAlpha = 0xFF)
{
#if defined(MOZ_CAIRO_GFX)
    *(PRUint32*)aDecoded = (aAlpha << 24) | (aRed << 16) | (aGreen << 8) | aBlue;
    aDecoded += 4;
#else // MOZ_CAIRO_GFX

#if defined(XP_MAC) || defined(XP_MACOSX)
    *aDecoded++ = 0; // Mac needs this padding byte
#endif
#ifdef USE_RGB
    *aDecoded++ = aRed;
    *aDecoded++ = aGreen;
    *aDecoded++ = aBlue;
#else
    *aDecoded++ = aBlue;
    *aDecoded++ = aGreen;
    *aDecoded++ = aRed;
#endif
#endif // MOZ_CAIRO_GFX
}

inline void SetPixel(PRUint8*& aDecoded, PRUint8 idx, colorTable* aColors)
{
    SetPixel(aDecoded, aColors[idx].red, aColors[idx].green, aColors[idx].blue);
}

/** Sets two (or one if aCount = 1) pixels
 * @param aDecoded where the data is stored. Will be moved 3 or 6 bytes,
 * depending on whether one or two pixels are written.
 * @param aData The values for the two pixels
 * @param aCount Current count. Is decremented by one or two. */
inline void Set4BitPixel(PRUint8*& aDecoded, PRUint8 aData,
                         PRUint32& aCount, colorTable* aColors)
{
    PRUint8 idx = aData >> 4;
    SetPixel(aDecoded, idx, aColors);
    if (--aCount > 0) {
        idx = aData & 0xF;
        SetPixel(aDecoded, idx, aColors);
        --aCount;
    }
}

#endif
