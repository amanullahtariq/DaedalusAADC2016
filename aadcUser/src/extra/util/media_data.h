#ifndef _MEDIA_DATA_H_
#define _MEDIA_DATA_H_

#include <string>

class MediaData {
  public:
    MediaData(int integer_data, std::string string_data);

    int get_integer_data() const;
    std::string get_string_data() const;

  private:
    int integer_data_;
    std::string string_data_;
};

#endif  // _MEDIA_DATA_H_