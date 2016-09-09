#ifndef READOCTFILE_H
#define READOCTFILE_H

//----------------------------------------------------------------------
template <typename T>
void readVector(const char* filepath, std::vector<T>& output) 
{
    std::FILE* input_file;
    input_file = std::fopen(filepath, "rb");

    if (input_file == NULL) {
      std::cerr << "Could not open file at " << filepath << std::endl;
      return;
    }

    // Get the file size in bytes
    std::fseek(input_file, 0, SEEK_END);
    int file_size = std::ftell(input_file);
    std::rewind(input_file);

    // Get the number of elements in the file
    uint32_t element_count = file_size / sizeof(T);

    // Read the file into data
    output.resize(element_count);
    uint32_t elements_read =
        std::fread(&(output[0]), sizeof(T), element_count, input_file);

    std::fclose(input_file);

    // Check to see if we read everything
    if (elements_read != element_count) {
      std::cerr << "Could not read everything from " << filepath << std::endl;
    }
}
//----------------------------------------------------------------------

#endif
