#pragma once

#include <cstdint>

#include <string>
#include <vector> 
#include <iostream>

#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/hdreg.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <cstring>

class LicenseManage
{
    struct HardInfo
    {
        std::string uuid;
        std::string model;
        std::string serialNo;
        std::string networkMac;
        std::string timetick;

        HardInfo()
        {
            uuid = "";
            model = "";
            serialNo = "";
            networkMac = "";
            timetick = "";
        }
    };

private:
    std::string license_code_path = "/home/iimt/.cfg/controlmaster/controlmaster/param/license/iimtkey.bin";
    std::string license_public_key = "/home/iimt/.cfg/controlmaster/controlmaster/param/license/public.key";
    HardInfo ohard_info_; // 原始存储信息
    HardInfo mhard_info_; // 密文的信息
    bool ok_{false};

    /**********************************************************
     * Description: 字符串分割
     * Input:    tokenstring: 待分割字符串
     *           delimiter: 分割符
     * Output:   无
     * Return:   std::vector<std::string>: 分割得到的所有字符串
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/
    std::vector<std::string> split(const std::string &tokenstring, char delimiter)
    {
        std::vector<std::string> tokens;

        size_t current_pos = 0;
        size_t pos = 0;
        while ((pos = tokenstring.find(delimiter, current_pos)) != std::string::npos)
        {
            tokens.push_back(tokenstring.substr(current_pos, pos - current_pos));
            current_pos = pos + 1;
        }

        tokens.push_back(tokenstring.substr(current_pos));
        return tokens;
    }

    /**********************************************************
     * Description: 获取磁盘信息
     * Input:    无
     * Output:   无
     * Return:   true: 成功获取磁盘信息
     *           false: 获取磁盘信息失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/    
    bool GetHardDiskInfo()
    {
        std::ifstream ifs;
        ifs.open("/home/iimt/.cfg/controlmaster/power_manage_uuid.md", std::ios::in);
        if (!ifs.is_open())
        {
            ////std::cout << "read error" << std::endl;
            return false;
        }
        std::string buf_uuid;
        while (std::getline(ifs, buf_uuid))
        {
            if (buf_uuid.find("UUID") != std::string::npos)
            {

                // /dev/sdb8: UUID="ed53ed01-cde8-483a-a7e6-b540d2c7e422" TYPE="swap" PARTUUID="fa30b15b-08"
                std::vector<std::string> v_str = split(buf_uuid, ' ');

                std::string tmp = v_str.at(1);
                tmp.erase(0, 6);
                int index = 0;
                while ((index = tmp.find('"', index)) != (int)std::string::npos)
                {
                    tmp.erase(index, 1);
                }
                ohard_info_.uuid = tmp;
                // //std::cout<<ohard_info_.uuid<<std::endl;
                break;
            }
        }
        ifs.close();

        /***********************/
        ifs.open("/home/iimt/.cfg/controlmaster/power_manage_serialid.md", std::ios::in);
        if (!ifs.is_open())
        {
            ////std::cout << "read error" << std::endl;
            return false;
        }
        std::string buf_harddisk;
        while (std::getline(ifs, buf_harddisk))
        {
            // E:ID_SERIAL=IPS325M6ZMC025600_AA000000000000001843
            int index = 0;
            if (buf_harddisk.find("ID_SERIAL") != std::string::npos)
            {
                while ((index = buf_harddisk.find(' ', index)) != (int)std::string::npos)
                {
                    buf_harddisk.erase(index, 1);
                }
                std::vector<std::string> v_str = split(buf_harddisk, '=');
                ohard_info_.serialNo = v_str.at(1);
                // //std::cout<<ohard_info_.serialNo<<std::endl;
                break;
            }
        }
        ifs.close();

        return true;
    }

    /**********************************************************
     * Description: 获取网络MAC信息
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/       
    void GetNetWorkInfo()
    {
        char host_name[256] = {0};
        gethostname(host_name, sizeof(host_name));
        struct hostent *host = gethostbyname(host_name);
        char ip_str[32] = {0};
        const char *ret = inet_ntop(host->h_addrtype, host->h_addr_list[0], ip_str, sizeof(ip_str));

        if (ret != NULL)
        {
            //            printf("hostname:%s\n",host_name);
            //            printf("Ip:%.20s\n",ip_str);
        }

        struct ifaddrs *ifAddrStruct = NULL;
        struct ifaddrs *ifa = NULL;

        for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (!ifa->ifa_addr)
            {
                continue;
            }

            if (ifa->ifa_addr->sa_family == AF_INET)
            {
                void *tmpAddPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
                char local_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, tmpAddPtr, local_ip, INET_ADDRSTRLEN);

                printf("%s IP:%s\n", ifa->ifa_name, local_ip);
            }
        }

        if (ifAddrStruct)
            freeifaddrs(ifAddrStruct);

        struct ifreq ifr;
        int fd = socket(AF_INET, SOCK_DGRAM, 0);

        char local_mac[128] = {0};

        strcpy(ifr.ifr_name, "enp3s0");
        if (0 == ioctl(fd, SIOCGIFHWADDR, &ifr))
        {
            char temp_str[10] = {0};
            memcpy(temp_str, ifr.ifr_hwaddr.sa_data, 6);
            sprintf(local_mac, "%02X-%02X-%02X-%02X-%02X-%02X", temp_str[0] & 0xff, temp_str[1] & 0xff, temp_str[2] & 0xff, temp_str[3] & 0xff, temp_str[4] & 0xff, temp_str[5] & 0xff);
        }
        // printf("local mac:%s\n",local_mac);

        ohard_info_.networkMac = local_mac;
        // //std::cout<<local_mac<<std::endl;
    }

    /**********************************************************
     * Description: 获取密钥
     * Input:    无
     * Output:   key: 获取的密钥
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/   
    void GetCode(std::string &key)
    {
        key = ohard_info_.uuid + "^" + ohard_info_.serialNo + "^" + ohard_info_.networkMac;
    }

    /**********************************************************
     * Description: 根据密钥解密获取固件内容
     * Input:    path: 许可证bin文件路径
     *           publickey_path: 许可证key文件路径
     * Output:   无
     * Return:   std::string: 读出的固件内容
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/       
    std::string ReadLicenseCode(const std::string path, const std::string publickey_path)
    {
        //std::ifstream inF(path.c_str(), std::ios::binary);
        std::ifstream inF;
        inF.open(path.c_str(), std::ios::binary);
        if (!inF.is_open())
        {
            ////std::cout << "read error" << std::endl;
            return "error";
        }
        // 将读写位置移动到文件末尾，获取文件大小
        inF.seekg(0, std::ios_base::end);
        long long firm_len = inF.tellg();
        // printf("size of firm: %lld\n", firm_len);

        // 将读写位置移动到文件开头，申请内存，将固件内容存入 pSrcBuf
        inF.seekg(0, std::ios_base::beg);
        auto *pSrcBuf = new char[firm_len];
        inF.read((char *)pSrcBuf, sizeof(char) * firm_len);
        std::string buf = DecryptByPubkeyFile(pSrcBuf, firm_len, publickey_path);

        // //std::cout<<buf<< std::endl;
        inF.close();
        delete[] pSrcBuf;
        return buf;
    }

    /**********************************************************
     * Description: 使用公钥文件解密
     * Input:    cipher: 固件内容
     *           len: 文件长度
     *           pub_filename: 公钥文件路径
     * Output:   无
     * Return:   std::string: 解密成功的明文数据
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/       
    std::string DecryptByPubkeyFile(char *cipher, uint32_t len,
                                    const std::string &pub_filename)
    {
        BIO *in = BIO_new(BIO_s_file());
        if (in == NULL)
        {
            //std::cout << "BIO_new failed" << std::endl;
            return "";
        }
        BIO_read_filename(in, pub_filename.c_str());

        RSA *rsa = PEM_read_bio_RSA_PUBKEY(in, NULL, NULL, NULL);
        BIO_free(in);
        if (rsa == NULL)
        {
            //std::cout << "PEM_read_bio_RSA_PUBKEY failed" << std::endl;
            return "";
        }

        int size = RSA_size(rsa);
        std::vector<char> data;
        data.resize(size);
        int ret =
            RSA_public_decrypt(len, (unsigned char *)cipher,
                               (unsigned char *)data.data(), rsa, RSA_PKCS1_PADDING);
        RSA_free(rsa);
        if (ret == -1)
        {
            //std::cout << "RSA_public_decrypt failed" << std::endl;
            return "";
        }
        std::string decrypt_data(data.begin(), data.end());
        return decrypt_data;
    }

public:
    /**********************************************************
     * Description: 许可证管理类构造函数
     * Input:    无
     * Output:   无
     * Return:   无
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/ 
    LicenseManage(/* args */)
    {
        system("udevadm info /dev/sda >>/home/iimt/.cfg/controlmaster/power_manage_serialid.md");
        system("blkid >>/home/iimt/.cfg/controlmaster/power_manage_uuid.md");
        std::string licensecode = ReadLicenseCode(license_code_path, license_public_key);
        int index = licensecode.find('\000');
        if(index != 0)
        	licensecode = licensecode.substr(0,index);
        // //std::cout<<licensecode<<std::endl;
        GetHardDiskInfo();
        GetNetWorkInfo();
        std::string code_;
        GetCode(code_);
        // //std::cout<<licensecode<<std::endl;
        //  //std::cout<<code_<<std::endl;
        system("rm /home/iimt/.cfg/controlmaster/power_manage_uuid.md");
        system("rm /home/iimt/.cfg/controlmaster/power_manage_serialid.md");
        int m = std::strcmp(licensecode.c_str(), code_.c_str());
        if (m == 0)
        {
            ok_ = true;
            //std::cout << "ok" << std::endl;
        }
        else
        {
            ok_ = false;
            //std::cout << "false" << std::endl;
        }
    }

    /**********************************************************
     * Description: 获取许可证
     * Input:    无
     * Output:   无
     * Return:   true: 成功获取许可证
     *           false: 获取许可证失败
     * Modify Date       Version       Author       Modification    
     * ---------------------------------------------------------
     * 2025/11/11        v1.0          liuzhiwen    代码初次注释  
     **********************************************************/     
    bool getLicense()
    {
        return ok_;
    }
};

