# perls2
PErception and Robotic Learning System v2

PERLS provides robot software infrastructure for projects within SVL. PERLS allows for a unified framework that enables robotics research on both simualted and real robots. PERLS2 is a redesign with the goal of offering only essential functionality for robotics research. Please refer to the PERLS Design Document for more information. 

PERLS is developed in a project-driven manner: tools and utilities developed in individual projects are standardized and reviewed before being merged into this repository.
TODO: Design Doc Link
TODO: API Link
TODO: Examples Links

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them

```
Give examples
```

### Installing
PERLS2 only supports python3 for the core libraries (python 2.7 is used for interfacing to robots that use ROS.)
PERLS2 only supports ubuntu 16.04 and later.
# On Ubuntu 16.04

1. Clone and install PERLs repo
    1. Clone repo: 
        
        `git clone https://github.com/StanfordVL/perls2.git`
    2. Create a virtual environment, e.g. 
        
        `virtualenv -p python3 perls2env`
    3. Source your virtual environment e.g. 
        
        `source perls2env/bin/activate'
    4. Go to the perls2 directory and install requirements
        
        `cd ~/perls2`
        
        `pip install -r requirements.txt .`
        * You could get an error like `error: numpy 1.11.0 is installed but numpy>=1.11.1 is required by set(['opencv-python'])`. In that case you may want to force to install the right version of numpy: `pip install --upgrade --force-reinstall numpy`
    5. Install perls2 
        
        `pip install -e .`

6. Run PERL demos
        `cd perls2/examples/simple_reach`
        `python run_simple_reach.py`


**TODO**
## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone whose code was used
* Inspiration
* etc

