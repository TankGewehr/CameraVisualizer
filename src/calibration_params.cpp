#include "calibration_params.h"

bool loadIntrinsic(const std::string &filename, cv::Mat &intrinsic, cv::Mat &distortion, cv::Size &image_size, bool undistorted)
{
	Json::Reader reader;
	Json::Value root;
	std::vector<double> intrinsic_vector, distortion_vector;
	const std::string intrinsic_key = undistorted ? "undistort_intrinsic" : "intrinsic";
	const std::string distortion_key = undistorted ? "undistort_distortion" : "distortion";

	std::ifstream is(filename, std::ios::binary);
	if (!is.is_open())
	{
		std::cout << "Error opening file:" << filename << std::endl;
		return false;
	}

	if (reader.parse(is, root))
	{
		// read intrinsic[9] or intrinsic[3][3]
		if (root[intrinsic_key].isNull() || root[intrinsic_key].type() != Json::arrayValue)
		{
			std::cout << "Error " << intrinsic_key << " type:" << filename << std::endl;
			is.close();
			return false;
		}

		if (root[intrinsic_key].size() == 3)
		{
			for (unsigned int i = 0; i < root[intrinsic_key].size(); i++)
			{
				if (root[intrinsic_key][i].isNull() || root[intrinsic_key][i].type() != Json::arrayValue)
				{
					std::cout << "Error " << intrinsic_key << " type:" << filename << ":" << i << std::endl;
					is.close();
					return false;
				}
				if (root[intrinsic_key][i].size() != 3)
				{
					std::cout << "Error " << intrinsic_key << " size:" << filename << ":" << i << std::endl;
					is.close();
					return false;
				}

				for (unsigned int j = 0; j < root[intrinsic_key][i].size(); j++)
				{
					double data = root[intrinsic_key][i][j].asDouble();
					intrinsic_vector.push_back(data);
				}
			}
		}
		else if (root[intrinsic_key].size() == 9)
		{
			for (unsigned int i = 0; i < root[intrinsic_key].size(); i++)
			{
				double data = root[intrinsic_key][i].asDouble();
				intrinsic_vector.push_back(data);
			}
		}
		else
		{
			std::cout << "Error " << intrinsic_key << " size:" << filename << std::endl;
			is.close();
			return false;
		}

		// read distortion[]
		if (root[distortion_key].isNull() || root[distortion_key].type() != Json::arrayValue)
		{
			std::cout << "Error " << distortion_key << " type:" << filename << std::endl;
			is.close();
			return false;
		}

		for (unsigned int i = 0; i < root[distortion_key].size(); i++)
		{
			double data = root[distortion_key][i].asDouble();
			distortion_vector.push_back(data);
		}

		// read image_size[2]
		if (root["image_size"].isNull() || root["image_size"].type() != Json::arrayValue)
		{
			std::cout << "Error image_size type:" << filename << std::endl;
			is.close();
			return false;
		}
		if (root["image_size"].size() != 2)
		{
			std::cout << "Error image_size size:" << filename << std::endl;
			is.close();
			return false;
		}

		image_size.width = root["image_size"][0].asInt();
		image_size.height = root["image_size"][1].asInt();
	}

	intrinsic = cv::Mat(intrinsic_vector).clone().reshape(1, 3);
	distortion = cv::Mat(distortion_vector).clone().reshape(1, 1);
	is.close();
	return true;
}

bool loadExtrinsic(const std::string &filename, cv::Mat &extrinsic)
{
	Json::Reader reader;
	Json::Value root;
	std::vector<double> extrinsic_vector, rotation, translation;

	std::ifstream is(filename, std::ios::binary);
	if (!is.is_open())
	{
		std::cout << "Error opening file:" << filename << std::endl;
		return false;
	}

	if (reader.parse(is, root))
	{
		// read rotation[9] or rotation[3][3]
		if (root["rotation"].isNull() || root["rotation"].type() != Json::arrayValue)
		{
			std::cout << "Error rotation type:" << filename << std::endl;
			is.close();
			return false;
		}
		if (root["rotation"].size() == 3)
		{
			for (unsigned int i = 0; i < root["rotation"].size(); i++)
			{
				if (root["rotation"][i].isNull() || root["rotation"][i].type() != Json::arrayValue)
				{
					std::cout << "Error rotation type:" << filename << ":" << i << std::endl;
					is.close();
					return false;
				}
				if (root["rotation"][i].size() != 3)
				{
					std::cout << "Error rotation size:" << filename << ":" << i << std::endl;
					is.close();
					return false;
				}

				for (unsigned int j = 0; j < root["rotation"][i].size(); j++)
				{
					double data = root["rotation"][i][j].asDouble();
					rotation.push_back(data);
				}
			}
		}
		else if (root["rotation"].size() == 9)
		{
			for (unsigned int i = 0; i < root["rotation"].size(); i++)
			{
				double data = root["rotation"][i].asDouble();
				rotation.push_back(data);
			}
		}
		else
		{
			std::cout << "Error rotation size:" << filename << std::endl;
			is.close();
			return false;
		}

		// read translation[3] or translation{x,y,z}
		if (!root["translation"].isNull() && root["translation"].type() == Json::arrayValue)
		{
			for (unsigned int i = 0; i < root["translation"].size(); i++)
			{
				double data = root["translation"][i].asDouble();
				translation.push_back(data);
			}
		}
		else if (!root["translation"].isNull() && root["translation"].type() == Json::objectValue)
		{
			double x = root["translation"]["x"].asDouble();
			double y = root["translation"]["y"].asDouble();
			double z = root["translation"]["z"].asDouble();
			translation.push_back(x);
			translation.push_back(y);
			translation.push_back(z);
		}
		else
		{
			std::cout << "Error translation type:" << filename << std::endl;
			is.close();
			return false;
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				extrinsic_vector.push_back(rotation[i * 3 + j]);
			}
			extrinsic_vector.push_back(translation[i]);
		}
		extrinsic_vector.push_back(0);
		extrinsic_vector.push_back(0);
		extrinsic_vector.push_back(0);
		extrinsic_vector.push_back(1);
	}

	extrinsic = cv::Mat(extrinsic_vector).clone().reshape(1, 4);
	is.close();
	return true;
}

bool saveExtrinsic(const std::string &filename, cv::Mat extrinsic)
{
	Json::Reader reader;
	Json::Value root;

	std::ifstream is(filename, std::ios::binary);
	if (!is.is_open())
	{
		std::cout << "Error opening file:" << filename << std::endl;
		return false;
	}

	if (reader.parse(is, root))
	{
		Json::Value rotation_obj;
		for (int i = 0; i < 9; i++)
		{
			rotation_obj.append(extrinsic.at<double>(i / 3, i % 3));
		}

		Json::Value translation_obj;
		for (int i = 0; i < 3; i++)
		{
			translation_obj.append(extrinsic.at<double>(i, 3));
		}

		root["rotation"] = rotation_obj;
		root["translation"] = translation_obj;
	}
	is.close();

	std::ofstream os;
	os.open(filename, std::ios::out);
	if (!os.is_open())
	{
		std::cout << "Error opening file:" << filename << std::endl;
		return false;
	}
	Json::StyledWriter sw;
	os << sw.write(root) << std::flush;
	os.close();

	return true;
}