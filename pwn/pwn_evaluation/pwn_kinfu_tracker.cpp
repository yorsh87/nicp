  // void FPSKinfuTracker::shrinkAndConvertRawDepth(pcl::gpu::PtrStepSz<const unsigned short>& dest_buffer, 
  // 						 RawDepthImage& dest_raw_buffer,
  // 						 const RawDepthImage& src_buffer, 
  // 						 int image_shrink) {
  //   int rows = src_buffer.rows;
  //   int cols = src_buffer.cols;   
  //   if(rows % image_shrink) { throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image rows"); }
  //   if(cols % image_shrink) { throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image cols"); }

  //   int drows = rows / image_shrink;
  //   int dcols = cols / image_shrink;

  //   dest_buffer.cols = dcols;
  //   dest_buffer.rows = drows;
  //   dest_buffer.step = dest_buffer.cols * dest_buffer.elemSize();
  //   _source_depth_data.resize(dest_buffer.cols * dest_buffer.rows);   
  //   std::fill(_source_depth_data.begin(), _source_depth_data.end(), 0);
  //   dest_raw_buffer.create(drows, dcols);
  //   dest_raw_buffer = 0;

  //   // avoid divisions and use a lookup table
  //   int lv = rows > cols ? rows:cols;
  //   int ttable[lv];
  //   for(int i = 0; i < lv; i++) { ttable[i] = i / image_shrink; }

  //   for(int r = 0; r < rows; r++) {
  //     const unsigned short* src_z_ptr = src_buffer.ptr<unsigned short>(r);
  //     int dr = ttable[r];
  //     unsigned short* dest_z_ptr = (unsigned short*) &_source_depth_data[r * cols];  
  //     unsigned short* dest_raw_z_ptr = dest_raw_buffer.ptr<unsigned short>(dr);
  //     int cc = 0;
  //     for(int c = 0; c < cols; c++) {
  // 	unsigned short src_z = *src_z_ptr;
  // 	src_z_ptr++;
  // 	if(src_z == 0) { continue; }
  // 	unsigned short& dest_z = *(dest_z_ptr + ttable[c]);
  //       unsigned short& dest_raw_z = *(dest_raw_z_ptr + ttable[c]);
  // 	if(!dest_z || dest_z < src_z) { 
  //         dest_z = src_z; 
  //         dest_raw_z = src_z; 
  //       }
  //     }
  //   }
  //   dest_buffer.data = &_source_depth_data[0];      
  // }
