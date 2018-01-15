# pfuclt_omni_dataset

This ROS package performs PF-UCLT on sensor msgs from the OMNI dataset (soccer robots dataset)

## Related publication

An extensive description of the method and results obtained can be found in our publication:

A. Ahmad; G. Lawless; P. Lima. **An Online Scalable Approach to Unified Multirobot Cooperative Localization and Object Tracking**. *IEEE Transactions on Robotics*, vol.PP, no.99, pp.1-16. **[PDF](https://ps.is.tuebingen.mpg.de/uploads_file/attachment/attachment/378/17-0058_02_MS.pdf)**.

## Requirements

The read_omni_dataset package from https://github.com/guilhermelawless/read_omni_dataset .

## Optimization

Some steps of the algorithm can be parallelized using OpenMP. To choose the number of threads to be used, set an environment variable in the terminal you're using to run the algorithm as: `export OMP_NUM_THREADS=<number of threads to use>`

## Dataset generation

Use the randgen_omni_dataset package from https://github.com/guilhermelawless/randgen_omni_dataset

## Citation

If you use PF-UCLT on an academic work, please cite:

    @ARTICLE{7964712, 
    author={A. Ahmad and G. Lawless and P. Lima}, 
    journal={IEEE Transactions on Robotics}, 
    title={An Online Scalable Approach to Unified Multirobot Cooperative Localization and Object Tracking}, 
    year={2017}, 
    volume={PP}, 
    number={99}, 
    pages={1-16}, 
    keywords={Cooperative perception;distributed robot systems;localization;sensor fusion;visual tracking}, 
    doi={10.1109/TRO.2017.2715342}, 
    ISSN={1552-3098}, 
    month={},}

## Documentation

Not very detailed but we have doxygen auto-generated documentation in https://guilhermelawless.github.io/pfuclt_omni_dataset/doc/html/c++/index.html
