import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
    title: string;
    description: ReactNode;
};

const FeatureList: FeatureItem[] = [
    {
        title: 'Structured Learning',
        description: (
            <>
                A clear path from fundamentals to advanced applications, with each chapter building on the last.
            </>
        ),
    },
    {
        title: 'Real-World Physical AI',
        description: (
            <>
                Explore practical examples and case studies from the world of robotics and embodied intelligence.
            </>
        ),
    },
    {
        title: 'ROS 2, Robotics, and AI Systems',
        description: (
            <>
                Dive deep into the tools and frameworks that power modern robotics and AI systems.
            </>
        ),
    },
];

function Feature({title, description}: FeatureItem) {
    return (
        <div className={clsx('col col--4', styles.featureCard)}>
            <div className={styles.featureCardHeader}>
                <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
            </div>
            <p className={styles.featureDescription}>{description}</p>
        </div>
    );
}

export default function HomepageFeatures(): ReactNode {
    return (
        <section className={styles.features}>
            <div className="container">
                <div className="row">
                    {FeatureList.map((props, idx) => (
                        <Feature key={idx} {...props}/>
                    ))}
                </div>
            </div>
        </section>
    );
}
