#ifndef locoman_service_1_MODULE_HPP_
#define locoman_service_1_MODULE_HPP_

#include <GYM/control_module.hpp>

#include "locoman_service_1_thread.h"
#include "locoman_service_1_constants.h"

/**
 * @brief locoman_service_1 module derived from generic_module
 * 
 * @author 
 */
class locoman_service_1_module : public control_module<locoman_service_1_thread> {
public:
    
    /**
     * @brief constructor: do nothing but construct the superclass
     * 
     */
    locoman_service_1_module(   int argc, 
                               char* argv[],
                               std::string module_prefix, 
                               int module_period, 
                               yarp::os::ResourceFinder rf ) : control_module<locoman_service_1_thread>(  argc, 
                                                                                            		argv, 
                                                                                            		module_prefix, 
                                                                                            		module_period,
                                                                                            		rf )
    {
    }
    
    /**
     * @brief overriden function to specify the custom params for the param helper
     * 
     * @return a vector of the custom params for the param helper
     */
    virtual std::vector< paramHelp::ParamProxyInterface* > custom_get_ph_parameters() 
    {
	// TODO: function skeleton
        std::vector<paramHelp::ParamProxyInterface *> custom_params;
        return custom_params;
    }
    
    
};

#endif
