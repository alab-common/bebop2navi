#ifndef _COVARIANCE_TIME_CACHE_H_
#define _COVARIANCE_TIME_CACHE_H_

#include "tf/tf.h"
#include <Eigen/Dense>
#include <set>

using Eigen::MatrixXd;

namespace uncertain_tf
{

typedef tf::Stamped<Eigen::MatrixXd> StampedCovariance;

class CovarianceStorage
{
public :

    CovarianceStorage() {};

    CovarianceStorage(const MatrixXd& data, ros::Time stamp); //, CompactFrameID frame_id);

    CovarianceStorage(const CovarianceStorage& rhs)
    {
        *this = rhs;
    }

    CovarianceStorage& operator=(const CovarianceStorage& rhs)
    {
       covariance_ = rhs.covariance_;
        stamp_ = rhs.stamp_;
        //frame_id_ = rhs.frame_id_;
        return *this;
    }

    bool operator< (const CovarianceStorage &b) const
    {
      return this->stamp_ < b.stamp_;
    }


    MatrixXd covariance_;
    ros::Time stamp_;

    //CompactFrameID frame_id_; // we do not store frame_ids since we always assume the connectivity at a given time is stored in the tf tree, and the covariances fit
};


class CovarianceTimeCache
{
public:
    static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
    static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
    static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage

    CovarianceTimeCache(ros::Duration max_storage_time = ros::Duration().fromNSec(DEFAULT_MAX_STORAGE_TIME));

    bool getData(ros::Time time, CovarianceStorage &data_out, std::string* error_str = 0);

    bool insertData(const CovarianceStorage& new_data);

    inline uint8_t findClosest(const CovarianceStorage*& one,const CovarianceStorage*& two, ros::Time target_time, std::string* error_str);

    inline void interpolate(const CovarianceStorage& one, const CovarianceStorage& two, ros::Time time, CovarianceStorage& output);

    void pruneList();

    typedef std::set<CovarianceStorage> L_CovarianceStorage;
    L_CovarianceStorage storage_;

    ros::Duration max_storage_time_;


};

}

#endif
