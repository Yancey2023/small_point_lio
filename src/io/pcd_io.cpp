/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#include "pcd_io.h"

/**
 * copy from https://github.com/isl-org/Open3D, modified to fit our need.
 * Open3D is open source under the MIT license: https://github.com/isl-org/Open3D/blob/main/LICENSE
 */
namespace io::pcd {

    constexpr size_t DEFAULT_IO_BUFFER_SIZE = 1024;

    std::vector<std::string> SplitString(const std::string &str,
                                         const std::string &delimiters = " ",
                                         bool trim_empty_str = true) {
        std::vector<std::string> tokens;
        std::string::size_type pos = 0, new_pos = 0, last_pos = 0;
        while (pos != std::string::npos) {
            pos = str.find_first_of(delimiters, last_pos);
            new_pos = (pos == std::string::npos ? str.length() : pos);
            if (new_pos != last_pos || !trim_empty_str) {
                tokens.push_back(str.substr(last_pos, new_pos - last_pos));
            }
            last_pos = new_pos + 1;
        }
        return tokens;
    }

    enum PCDDataType {
        PCD_DATA_ASCII = 0,
        PCD_DATA_BINARY = 1,
        PCD_DATA_BINARY_COMPRESSED = 2
    };

    struct PCLPointField {// NOLINT(cppcoreguidelines-pro-type-member-init)
    public:
        std::string name;
        int size;
        char type;
        int count;
        // helper variable
        int count_offset;
        int offset;
    };

    struct PCDHeader {// NOLINT(cppcoreguidelines-pro-type-member-init)
    public:
        std::string version;
        std::vector<PCLPointField> fields;
        int width;
        int height;
        int points;
        PCDDataType datatype;
        std::string viewpoint;
        // helper variables
        int elementnum;
        int pointsize;
        bool has_points;
    };

    bool CheckHeader(PCDHeader &header) {
        if (header.points <= 0 || header.pointsize <= 0) {
            return false;
        }
        if (header.fields.size() == 0 || header.pointsize <= 0) {
            return false;
        }
        bool has_x = false;
        bool has_y = false;
        bool has_z = false;
        for (const auto &field: header.fields) {
            if (field.name == "x") {
                has_x = true;
            } else if (field.name == "y") {
                has_y = true;
            } else if (field.name == "z") {
                has_z = true;
            }
        }
        header.has_points = (has_x && has_y && has_z);
        if (!header.has_points) {
            return false;
        }
        return true;
    }

    bool ReadPCDHeader(FILE *file, PCDHeader &header) {
        char line_buffer[DEFAULT_IO_BUFFER_SIZE];
        size_t specified_channel_count = 0;

        while (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, file)) {
            std::string line(line_buffer);
            if (line == "") {
                continue;
            }
            std::vector<std::string> st = SplitString(line, "\t\r\n ");
            std::stringstream sstream(line);
            sstream.imbue(std::locale::classic());
            std::string line_type;
            sstream >> line_type;
            if (line_type.substr(0, 1) == "#") {
            } else if (line_type.substr(0, 7) == "VERSION") {
                if (st.size() >= 2) {
                    header.version = st[1];
                }
            } else if (line_type.substr(0, 6) == "FIELDS" ||
                       line_type.substr(0, 7) == "COLUMNS") {
                specified_channel_count = st.size() - 1;
                if (specified_channel_count == 0) {
                    return false;
                }
                header.fields.resize(specified_channel_count);
                int count_offset = 0, offset = 0;
                for (size_t i = 0; i < specified_channel_count;
                     i++, count_offset += 1, offset += 4) {
                    header.fields[i].name = st[i + 1];
                    header.fields[i].size = 4;
                    header.fields[i].type = 'F';
                    header.fields[i].count = 1;
                    header.fields[i].count_offset = count_offset;
                    header.fields[i].offset = offset;
                }
                header.elementnum = count_offset;
                header.pointsize = offset;
            } else if (line_type.substr(0, 4) == "SIZE") {
                if (specified_channel_count != st.size() - 1) {
                    return false;
                }
                int offset = 0, col_type = 0;
                for (size_t i = 0; i < specified_channel_count;
                     i++, offset += col_type) {
                    sstream >> col_type;
                    header.fields[i].size = col_type;
                    header.fields[i].offset = offset;
                }
                header.pointsize = offset;
            } else if (line_type.substr(0, 4) == "TYPE") {
                if (specified_channel_count != st.size() - 1) {
                    return false;
                }
                for (size_t i = 0; i < specified_channel_count; i++) {
                    header.fields[i].type = st[i + 1].c_str()[0];
                }
            } else if (line_type.substr(0, 5) == "COUNT") {
                if (specified_channel_count != st.size() - 1) {
                    return false;
                }
                int count_offset = 0, offset = 0, col_count = 0;
                for (size_t i = 0; i < specified_channel_count; i++) {
                    sstream >> col_count;
                    header.fields[i].count = col_count;
                    header.fields[i].count_offset = count_offset;
                    header.fields[i].offset = offset;
                    count_offset += col_count;
                    offset += col_count * header.fields[i].size;
                }
                header.elementnum = count_offset;
                header.pointsize = offset;
            } else if (line_type.substr(0, 5) == "WIDTH") {
                sstream >> header.width;
            } else if (line_type.substr(0, 6) == "HEIGHT") {
                sstream >> header.height;
                header.points = header.width * header.height;
            } else if (line_type.substr(0, 9) == "VIEWPOINT") {
                if (st.size() >= 2) {
                    header.viewpoint = st[1];
                }
            } else if (line_type.substr(0, 6) == "POINTS") {
                sstream >> header.points;
            } else if (line_type.substr(0, 4) == "DATA") {
                header.datatype = PCD_DATA_ASCII;
                if (st.size() >= 2) {
                    if (st[1].substr(0, 17) == "binary_compressed") {
                        header.datatype = PCD_DATA_BINARY_COMPRESSED;
                    } else if (st[1].substr(0, 6) == "binary") {
                        header.datatype = PCD_DATA_BINARY;
                    }
                }
                break;
            }
        }
        if (!CheckHeader(header)) {
            return false;
        }
        return true;
    }

    float UnpackBinaryPCDElement(const char *data_ptr, const char type, const int size) {
        const char type_uppercase = std::toupper(type, std::locale());
        if (type_uppercase == 'I') {
            if (size == 1) {
                std::int8_t data;
                memcpy(&data, data_ptr, sizeof(data));
                return (float) data;
            } else if (size == 2) {
                std::int16_t data;
                memcpy(&data, data_ptr, sizeof(data));
                return (float) data;
            } else if (size == 4) {
                std::int32_t data;
                memcpy(&data, data_ptr, sizeof(data));
                return (float) data;
            } else {
                return 0.0f;
            }
        } else if (type_uppercase == 'U') {
            if (size == 1) {
                std::uint8_t data;
                memcpy(&data, data_ptr, sizeof(data));
                return (float) data;
            } else if (size == 2) {
                std::uint16_t data;
                memcpy(&data, data_ptr, sizeof(data));
                return (float) data;
            } else if (size == 4) {
                std::uint32_t data;
                memcpy(&data, data_ptr, sizeof(data));
                return (float) data;
            } else {
                return 0.0f;
            }
        } else if (type_uppercase == 'F') {
            if (size == 4) {
                float data;
                memcpy(&data, data_ptr, sizeof(data));
                return (float) data;
            } else {
                return 0.0f;
            }
        }
        return 0.0f;
    }

    float UnpackASCIIPCDElement(const char *data_ptr, const char type, const int size) {
        char *end;
        const char type_uppercase = std::toupper(type, std::locale());
        if (type_uppercase == 'I') {
            return (float) std::strtol(data_ptr, &end, 0);
        } else if (type_uppercase == 'U') {
            return (float) std::strtoul(data_ptr, &end, 0);
        } else if (type_uppercase == 'F') {
            return std::strtof(data_ptr, &end);
        }
        return 0.0f;
    }

    bool ReadPCDData(FILE *file, const PCDHeader &header, std::vector<Eigen::Vector3f> &pointcloud) {
        // The header should have been checked
        if (header.has_points) {
            pointcloud.resize(header.points);
        } else {
            return false;
        }
        if (header.datatype == PCD_DATA_ASCII) {
            char line_buffer[DEFAULT_IO_BUFFER_SIZE];
            int idx = 0;
            while (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, file) && idx < header.points) {
                std::string line(line_buffer);
                std::vector<std::string> strs = SplitString(line, "\t\r\n ");
                if ((int) strs.size() < header.elementnum) {
                    continue;
                }
                for (const auto &field: header.fields) {
                    if (field.name == "x") {
                        pointcloud[idx].x() = UnpackASCIIPCDElement(strs[field.count_offset].c_str(), field.type, field.size);
                    } else if (field.name == "y") {
                        pointcloud[idx].y() = UnpackASCIIPCDElement(strs[field.count_offset].c_str(), field.type, field.size);
                    } else if (field.name == "z") {
                        pointcloud[idx].z() = UnpackASCIIPCDElement(strs[field.count_offset].c_str(), field.type, field.size);
                    }
                }
                idx++;
            }
        } else if (header.datatype == PCD_DATA_BINARY) {
            std::unique_ptr<char[]> buffer(new char[header.pointsize]);
            for (int i = 0; i < header.points; i++) {
                if (fread(buffer.get(), header.pointsize, 1, file) != 1) {
                    pointcloud.clear();
                    return false;
                }
                for (const auto &field: header.fields) {
                    if (field.name == "x") {
                        pointcloud[i].x() = UnpackBinaryPCDElement(buffer.get() + field.offset, field.type, field.size);
                    } else if (field.name == "y") {
                        pointcloud[i].y() = UnpackBinaryPCDElement(buffer.get() + field.offset, field.type, field.size);
                    } else if (field.name == "z") {
                        pointcloud[i].z() = UnpackBinaryPCDElement(buffer.get() + field.offset, field.type, field.size);
                    }
                }
            }
        } else if (header.datatype == PCD_DATA_BINARY_COMPRESSED) {
            std::uint32_t compressed_size;
            std::uint32_t uncompressed_size;
            if (fread(&compressed_size, sizeof(compressed_size), 1, file) != 1) {
                pointcloud.clear();
                return false;
            }
            if (fread(&uncompressed_size, sizeof(uncompressed_size), 1, file) != 1) {
                pointcloud.clear();
                return false;
            }
            std::unique_ptr<char[]> buffer_compressed(new char[compressed_size]);
            if (fread(buffer_compressed.get(), 1, compressed_size, file) != compressed_size) {
                pointcloud.clear();
                return false;
            }
            std::unique_ptr<char[]> buffer(new char[uncompressed_size]);
            if (lzf_decompress(buffer_compressed.get(),
                               (unsigned int) compressed_size, buffer.get(),
                               (unsigned int) uncompressed_size) != uncompressed_size) {
                pointcloud.clear();
                return false;
            }
            for (const auto &field: header.fields) {
                const char *base_ptr = buffer.get() + field.offset * header.points;
                if (field.name == "x") {
                    for (int i = 0; i < header.points; i++) {
                        pointcloud[i].x() = UnpackBinaryPCDElement(base_ptr + i * field.size * field.count, field.type, field.size);
                    }
                } else if (field.name == "y") {
                    for (int i = 0; i < header.points; i++) {
                        pointcloud[i].y() = UnpackBinaryPCDElement(base_ptr + i * field.size * field.count, field.type, field.size);
                    }
                } else if (field.name == "z") {
                    for (int i = 0; i < header.points; i++) {
                        pointcloud[i].z() = UnpackBinaryPCDElement(base_ptr + i * field.size * field.count, field.type, field.size);
                    }
                }
            }
        }
        return true;
    }

    bool GenerateHeader(const std::vector<Eigen::Vector3f> &pointcloud, const bool write_ascii, const bool compressed, PCDHeader &header) {
        if (pointcloud.empty()) {
            return false;
        }
        header.version = "0.7";
        header.width = (int) pointcloud.size();
        header.height = 1;
        header.points = header.width;
        header.fields.clear();
        PCLPointField field;
        field.type = 'F';
        field.size = 4;
        field.count = 1;
        field.name = "x";
        header.fields.push_back(field);
        field.name = "y";
        header.fields.push_back(field);
        field.name = "z";
        header.fields.push_back(field);
        header.elementnum = 3;
        header.pointsize = 12;
        if (write_ascii) {
            header.datatype = PCD_DATA_ASCII;
        } else {
            if (compressed) {
                header.datatype = PCD_DATA_BINARY_COMPRESSED;
            } else {
                header.datatype = PCD_DATA_BINARY;
            }
        }
        return true;
    }

    bool WritePCDHeader(FILE *file, const PCDHeader &header) {
        fprintf(file, "# .PCD v%s - Point Cloud Data file format\n",
                header.version.c_str());
        fprintf(file, "VERSION %s\n", header.version.c_str());
        fprintf(file, "FIELDS");
        for (const auto &field: header.fields) {
            fprintf(file, " %s", field.name.c_str());
        }
        fprintf(file, "\n");
        fprintf(file, "SIZE");
        for (const auto &field: header.fields) {
            fprintf(file, " %d", field.size);
        }
        fprintf(file, "\n");
        fprintf(file, "TYPE");
        for (const auto &field: header.fields) {
            fprintf(file, " %c", field.type);
        }
        fprintf(file, "\n");
        fprintf(file, "COUNT");
        for (const auto &field: header.fields) {
            fprintf(file, " %d", field.count);
        }
        fprintf(file, "\n");
        fprintf(file, "WIDTH %d\n", header.width);
        fprintf(file, "HEIGHT %d\n", header.height);
        fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
        fprintf(file, "POINTS %d\n", header.points);

        switch (header.datatype) {
            case PCD_DATA_BINARY:
                fprintf(file, "DATA binary\n");
                break;
            case PCD_DATA_BINARY_COMPRESSED:
                fprintf(file, "DATA binary_compressed\n");
                break;
            case PCD_DATA_ASCII:
            default:
                fprintf(file, "DATA ascii\n");
                break;
        }
        return true;
    }

    bool WritePCDData(FILE *file,
                      const PCDHeader &header,
                      const std::vector<Eigen::Vector3f> &pointcloud,
                      const WritePointCloudOption &params) {
        if (header.datatype == PCD_DATA_ASCII) {
            for (const auto &point: pointcloud) {
                fprintf(file, "%.10g %.10g %.10g\n", point(0), point(1), point(2));
                fprintf(file, "\n");
            }
        } else if (header.datatype == PCD_DATA_BINARY) {
            std::unique_ptr<float[]> data(new float[header.elementnum]);
            for (const auto &point: pointcloud) {
                data[0] = (float) point(0);
                data[1] = (float) point(1);
                data[2] = (float) point(2);
                fwrite(data.get(), sizeof(float), header.elementnum, file);
            }
        } else if (header.datatype == PCD_DATA_BINARY_COMPRESSED) {
            int strip_size = header.points;
            auto buffer_size = (std::uint32_t)(header.elementnum * header.points);
            std::unique_ptr<float[]> buffer(new float[buffer_size]);
            std::unique_ptr<float[]> buffer_compressed(new float[buffer_size * 2]);
            for (size_t i = 0; i < pointcloud.size(); i++) {
                const auto &point = pointcloud[i];
                buffer[0 * strip_size + i] = (float) point(0);
                buffer[1 * strip_size + i] = (float) point(1);
                buffer[2 * strip_size + i] = (float) point(2);
            }
            std::uint32_t buffer_size_in_bytes = buffer_size * sizeof(float);
            std::uint32_t size_compressed = lzf_compress(buffer.get(), buffer_size_in_bytes,
                                                         buffer_compressed.get(), buffer_size_in_bytes * 2);
            if (size_compressed == 0) {
                return false;
            }
            fwrite(&size_compressed, sizeof(size_compressed), 1, file);
            fwrite(&buffer_size_in_bytes, sizeof(buffer_size_in_bytes), 1, file);
            fwrite(buffer_compressed.get(), 1, size_compressed, file);
        }
        return true;
    }

    bool read_pcd(const std::string &filename, std::vector<Eigen::Vector3f> &pointcloud) {
        PCDHeader header;
        FILE *file = fopen(filename.c_str(), "rb");
        if (file == nullptr) {
            return false;
        }
        if (!ReadPCDHeader(file, header)) {
            fclose(file);
            return false;
        }
        if (!ReadPCDData(file, header, pointcloud)) {
            fclose(file);
            return false;
        }
        fclose(file);
        return true;
    }

    bool write_pcd(const std::string &filename, const std::vector<Eigen::Vector3f> &pointcloud, const WritePointCloudOption &params) {
        PCDHeader header;
        if (!GenerateHeader(pointcloud, bool(params.write_ascii),
                            bool(params.compressed), header)) {
            return false;
        }
        FILE *file = fopen(filename.c_str(), "wb");
        if (file == nullptr) {
            return false;
        }
        if (!WritePCDHeader(file, header)) {
            fclose(file);
            return false;
        }
        if (!WritePCDData(file, header, pointcloud, params)) {
            fclose(file);
            return false;
        }
        fclose(file);
        return true;
    }

}// namespace io::pcd
